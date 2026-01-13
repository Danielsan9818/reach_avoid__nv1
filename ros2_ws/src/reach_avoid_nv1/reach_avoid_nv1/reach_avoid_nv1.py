from drone_control.drone_control.optimal_control import Optimal_Control
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import FullState, StringArray, Position
from std_msgs.msg import Bool
from rclpy.duration import Duration
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Pose, Twist, PoseStamped
from scipy.linalg import expm, logm
import time
import numpy as np

class reach_avoid_nv1(Node):
    def __init__(self):
        """
            Node that sends the crazyflie to a desired position
            The desired position comes from the distortion of a circle
        """
        super().__init__('reach_avoid_nv1')
        self.info = self.get_logger().info
        self.info('reach avoid game node has been started.')
        self.declare_parameter('r', '1.')
        self.declare_parameter('robots', ['C04', 'C13', 'C05'])#,'C14','C20']) 
        self.declare_parameter('number_of_agents', '3')
        self.declare_parameter('phi_dot', '0.8')
  
        self.robots = self.get_parameter('robots').value
        self.n_agents  = int(self.get_parameter('number_of_agents').value)
        self.r  = float(self.get_parameter('r').value)
        self.k_phi  = 8#float(self.get_parameter('k_phi').value)
        self.phi_dot  = float(self.get_parameter('phi_dot').value)
        self.initial_phase = 0
        self.reboot_client = self.create_client(Empty, self.robots + '/reboot')

        self.has_initial_pose = [False] * self.n_agents
        self.has_final = [False] * self.n_agents
        self.land_flag = [False] * self.n_agents
        

        self.final_pose = np.zeros((self.n_agents, 3))
        self.current_pos = np.zeros((self.n_agents, 3))
        self.initial_pose = np.zeros((self.n_agents, 3))
        self.hover_height = 0.9
        self.leader = None
        self.follower = None
        self.Rot_des = np.eye(3)
        self.target_r = np.zeros(3)
        self.timer_period = 0.1

        self.i_landing = 0
        self.i_takeoff = 0

        self.phases = np.zeros(self.n_agents)

        self.phi_cur = Float32()
        self.phase_pub = self.create_publisher(Float32,'/'+ self.robots + '/phase', 1)

        self.state = 0
        #0-take-off, 1-hover, 2-encirclement, 3-landing

        self.create_subscription(
            Bool,
            '/landing',
            self._landing_callback,
            10)
        self.create_subscription(
            Bool,
            '/encircle',
            self._encircle_callback,
            10)

        qos_profile = QoSProfile(reliability =QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=0, nanoseconds=0))

        self.create_subscription(
            NamedPoseArray, "/poses",
            self._poses_changed, qos_profile
        )

        while (not self.has_initial_pose):
            rclpy.spin_once(self, timeout_sec=0.1)


        self.info(f"agents phases: {self.phases}")

        for robot in self.robots:
            self.position_pub[robot] = self.create_publisher(Position,'/'+ self.robots[robot] + '/cmd_position', 10) #create list with publishers for robot in self.robots


        #initiating some variables


        # input("Press Enter to takeoff")
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):

        try:
            if self.state == 0:
                if self.has_initial_pose:
                    # self.phase_pub.publish(self.phi_cur)
                    self.takeoff()

            elif self.state == 1:
                self.hover() 
            
            elif self.state == 2: 
                # target_r = self.interpolated_points[self.current_point_index] #here you calculate your desired position
                self.Control_loop() #call your control loop that returns the desired position
                for i,robot in enumerate(self.robots):
                    self.send_position(self.send_positions_pur_eva[i],robot)
                # if np.linalg.norm(self.current_pos-target_r) < 0.05:
            
            elif self.state == 3:
                if self.has_final:
                    self.landing()

                    if self.i_landing < len(self.t_landing)-1:
                        self.i_landing += 1
                    else:
                        self.reboot()
                        self.info('Exiting circle node')  
                        self.destroy_node()
                        rclpy.shutdown()             
        except KeyboardInterrupt:
            self.info('Exiting open loop command node')

            #self.publishers[i].publish(msg)
    
    def _poses_changed(self, msg):
        """
        Topic update callback to the motion capture lib's
           poses topic to send through the external position
           to the crazyflie 
        """
        for i,robot in enumerate(self.robots):
            for pose in msg.poses:
                    if pose.name == robot:
                        robot_pose = pose.pose
                        break
            if not self.has_initial_pose[i]:
                self.initial_pose[i, 0] = robot_pose.position.x
                self.initial_pose[i, 1] = robot_pose.position.y
                self.initial_pose[i, 2] = robot_pose.position.z   
                self.initial_phase = np.mod(np.arctan2(self.initial_pose[i, 1], self.initial_pose[i, 0]),2*np.pi)   
                self.takeoff_traj(4,i)
                self.has_initial_pose[i] = True    
                
            elif not self.land_flag[i]:

                self.current_pos[i,0] = robot_pose.position.x
                self.current_pos[i,1] = robot_pose.position.y
                self.current_pos[i,2] = robot_pose.position.z

            elif (self.has_final == False) and (self.land_flag[i] == True):
                
                # self.final_pose = np.zeros(3)
                self.info("Landing...")
                self.final_pose[i,0] = robot_pose.position.x
                self.final_pose[i,1] = robot_pose.position.y
                self.final_pose[i,2] = robot_pose.position.z
                self.landing_traj(2,i)
                self.has_final = True



    def takeoff(self,robot):
        self.send_position(self.r_takeoff[:,self.i_takeoff],robot)
        #self.info(f"Publishing to {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
        if self.i_takeoff < len(self.t_takeoff)-1:
            self.i_takeoff += 1
        else:
            self.state = 1

    def takeoff_traj(self,t_max,i):
        #takeoff trajectory
        self.t_takeoff = np.arange(0,t_max,self.timer_period)
        self.r_takeoff = np.zeros((3,len(self.t_takeoff))) 
        self.r_takeoff[0,:] += self.initial_pose[0]*np.ones(len(self.t_takeoff))
        self.r_takeoff[1,:] += self.initial_pose[1]*np.ones(len(self.t_takeoff))
        self.r_takeoff[2,:] = self.hover_height[i]*(self.t_takeoff/t_max)

    def landing_traj(self,t_max,i):
        #landing trajectory
        self.t_landing = np.arange(t_max,0.1,-self.timer_period)
        self.i_landing = 0
        self.r_landing = np.zeros((3,len(self.t_landing)))
        self.r_landing[0,:] += self.final_pose[0]*np.ones(len(self.t_landing))
        self.r_landing[1,:] += self.final_pose[1]*np.ones(len(self.t_landing))
        self.r_landing[2,:] = self.hover_height[i]*(self.t_landing/t_max)
    
    def _landing_callback(self, msg):
        self.land_flag = msg.data
        self.state = 3

    def _encircle_callback(self, msg):
        self.state = 2

    def hover(self):
        msg = Position()
        msg.x = self.initial_pose[0]
        msg.y = self.initial_pose[1]
        msg.z = self.hover_height
        self.position_pub.publish(msg)


    def landing(self):
        for robot in self.robots:
            self.send_position(self.r_landing[:,self.i_landing],robot)

    def reboot(self):
        req = Empty.Request()
        self.reboot_client.call_async(req)
        time.sleep(1.0)    

    def send_position(self,r,robot):
        msg = Position()
        msg.x = float(r[0])
        msg.y = float(r[1])
        msg.z = float(r[2])

        self.position_pub[robot].publish(msg)


    def interpolate(self, p0, p1, n):
        p0 = np.array(p0)
        p1 = np.array(p1)
        return [tuple(p0 + (p1 - p0) * t) for t in np.linspace(0, 1, n)]
    
    ## addition for pursuer evader scenario
    
    def game_parametrs(self):
        self.number_pursuers = self.n_agents -1

        self.dt=self.timer_period
        self.evader_speed = np.array([5]) # evader at 5 cm/s  for initial experiments
        # self.pursuers_speed = np.array([20,40,30,21,32])
        self.pursuers_speed = np.ones(self.number_pursuers)*6 #each pursuer at 6 cm/s for initial experiments
        # self.r = np.array([30,15,20,50,25])
        self.r = np.ones(self.number_pursuers)*50  #capture radius set to 50 cm for safety

        self.noisy_speedp = self.pursuers_speed
        self.noisy_speede = self.evader_speed



        self.which_area = 1  #1- ellipsoid, 2 - lp ball
        if self.which_area==1:

            center = np.array([0, 0, 0])  # x0, y0, z0
            a, b, c = 8, 8, 2                   # ellipse axes lengths
            self.par_ellipsoide = np.array([a,b,c])

        elif (self.which_area==2):
            center = np.array([0, 0, 0])  # x0, y0, z0
            a,b,c,p = 8,5,10,3 # scale x,y,x and exponential
            self.par_ellipsoide = np.array([a,b,c,p])                   # lp- ball 

        else:

            center = np.array([0, 0, 0])  # x0, y0, z0
            a, b, c = 8, 8, 2                   # ellipse axes lengths
            self.par_ellipsoide = np.array([a,b,c])

        #min 5cm/s max 100cm/s
        #ruido posicao 0.1
        #ruido velocidade metade da velocidade
        self.failure_rate = 0.0
        self.mode=1
        self.x0=[0.0,0.0,0.0]

    def Control_loop(self):

        self.pos_pursuers = self.current_pos[:self.number_pursuers,:]
        self.pos_evader = self.current_pos[self.number_pursuers,:]

        if self.pursuer_win(self.pos_evader,self.pos_pursuers,self.r):
            self.info("Pursuers win!")
            self.state = 3

        if self.evader_win(self.pos_evader):
            self.info("Evader wins!")
            self.state = 3

        self.vel_pursuer,self.vel_evader,self.x0 = Optimal_Control(self.pos_pursuers,self.pos_evader,self.r,self.pursuers_speed,self.evader_speed,self.mode,self.dt,self.x0,self.noisy_speedp,self.noisy_speede,self.par_ellipsoide,self.which_area)

        self.send_positions_pur_eva = np.append(self.vel_pursuer,self.vel_evader) + self.current_pos

        self.send_positions_pur_eva = self.env_limitations(np.array(self.send_positions_pur_eva))


        # return self.send_positions_pur_eva
        # # Publish Pursuer commands
        # for i in range(self.number_pursuers):
        #     cmd = Twist()
        #     cmd.linear.x, cmd.linear.y, cmd.linear.z = self.vel_pursuer[i]
        #     self.p_pubs[i].publish(cmd)

        # # Publish Evader command
        # cmd_e = Twist()
        # cmd_e.linear.x, cmd_e.linear.y, cmd_e.linear.z = self.vel_evader 
        # self.e_pub.publish(cmd_e)

    def env_limitations(self,pos):
        for i in range(pos.shape[0]):
            pos[i,0] = np.clip(pos[i,0],-150,150) #clip to 200 cm in each direction
            pos[i,1] = np.clip(pos[i,1],-200,200) #clip to 200 cm in each direction
            pos[i,2] = np.clip(pos[i,2],0,150) #clip to 150 cm in z direction
        return pos
    
    def pursuer_win(self,pos_evader,pos_pursuers,r):
        for i in range(pos_pursuers.shape[0]):
            if np.linalg.norm(pos_evader - pos_pursuers[i,:]) < r[i]:
                return True
        return False
    
    def evader_win(self,pos_evader):
        if self.which_area==1:
            gameValue = (pos_evader[0]**2) / self.par_ellipsoide[0]**2 + (pos_evader[1]**2) / self.par_ellipsoide[1]**2 + (pos_evader[2]**2) / self.par_ellipsoide[2]**2 - 1
        elif (self.which_area==2):
            gameValue = (np.abs(pos_evader[0] / self.par_ellipsoide[0]))**self.par_ellipsoide[3] + (np.abs(pos_evader[1] / self.par_ellipsoide[1]))**self.par_ellipsoide[3]  + (np.abs(pos_evader[2] / self.par_ellipsoide[2]))**self.par_ellipsoide[3]  - 1
        else:
            gameValue = (pos_evader[0]**2) / self.par_ellipsoide[0]**2 + (pos_evader[1]**2) / self.par_ellipsoide[1]**2 + (pos_evader[2]**2) / self.par_ellipsoide[2]**2 - 1
            
        if gameValue<1:
            return True
        return False

def main():
    rclpy.init()
    ra = reach_avoid_nv1()
    rclpy.spin(ra)
    ra.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
