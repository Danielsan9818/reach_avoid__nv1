from drone_control.optimal_control import Optimal_Control
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np

class DroneController(Node):
    def __init__(self):
        super().__init__('game_controller')
        
        # Storage for all drone positions
        self.pos_pursuers = np.zeros((4, 3)) # 4 drones, (x, y, z)
        self.pos_evader = np.zeros(3)        # 1 evader, (x, y, z)
        
        # 1. Subscribers for 4 Pursuers
        self.p_subs = []
        for i in range(4):
            self.p_subs.append(self.create_subscription(
                PoseStamped, f'/pursuer_{i}/position', 
                lambda msg, idx=i: self.p_callback(msg, idx), 10))
        
        # 2. Subscriber for 1 Evader
        self.e_sub = self.create_subscription(
            PoseStamped, '/evader/position', self.e_callback, 10)
            
        # 3. Publishers for 5 Drones
        self.p_pubs = [self.create_publisher(Twist, f'/pursuer_{i}/cmd_vel', 10) for i in range(4)]
        self.e_pub = self.create_publisher(Twist, '/evader/cmd_vel', 10)

        # 4. Timer to run the control loop (e.g., 20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)


    def p_callback(self, msg, idx):
        self.pos_pursuers[idx] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def e_callback(self, msg):
        self.pos_evader = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def control_loop(self):
        # Now run your Optimal_Control using the stored positions
        # vel_pursuer (should be 4x3), vel_evader (1x3)
        self.vel_pursuer,self.vel_evader,self.x0 = Optimal_Control(self.pos_pursuers,self.pos_evader,self.r,self.pursuers_speed,self.evader_speed,self.mode,self.dt,x0,self.noisy_speedp,self.noisy_speede,self.par_ellipsoide,self.which_area)

        # Publish Pursuer commands
        for i in range(4):
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.linear.z = self.vel_pursuer[i]
            self.p_pubs[i].publish(cmd)

        # Publish Evader command
        cmd_e = Twist()
        cmd_e.linear.x, cmd_e.linear.y, cmd_e.linear.z = self.vel_evader 
        self.e_pub.publish(cmd_e)


    def game_parametrs(self):
        self.number_pursuers = 4

        self.which_area = 2
        if self.which_area==1:

            center = np.array([0, 0, 0])  # x0, y0, z0
            a, b, c = 8, 8, 2                   # ellipse axes lengths
            self.par_ellipsoide = np.array([a,b,c])
            # theta = np.radians(0)        # rotation angle around Z axis
            # u = np.linspace(0, 2 * np.pi, 100)
            # v = np.linspace(0, np.pi, 50)

            # x =  np.outer(np.cos(u), np.sin(v))
            # y = b * np.outer(np.sin(u), np.sin(v))
            # z = c * np.outer(np.ones_like(u), np.cos(v))

        elif (self.which_area==2):
            center = np.array([0, 0, 0])  # x0, y0, z0
            a,b,c,p = 8,5,10,3 # scale x,y,x and exponential
            self.par_ellipsoide = np.array([a,b,c,p])                   # lp- ball 
            # par_ellipsoide = np.array([a,b,c])

            # theta = np.radians(0)        # rotation angle around Z axis
            # u = np.linspace(0, 2*np.pi, 100)
            # v = np.linspace(-np.pi/2, np.pi/2, 50)

            # U, V = np.meshgrid(u, v)

            # # Parametric formula for Lp-ball surface (smooth convex bodies)
            # x =  a * np.sign(np.cos(V) * np.cos(U)) * np.abs(np.cos(V) * np.cos(U))**(1/self.par_ellipsoide[3])
            # y =  b * np.sign(np.cos(V) * np.sin(U)) * np.abs(np.cos(V) * np.sin(U))**(1/self.par_ellipsoide[3])
            # z =  c * np.sign(np.sin(V)) * np.abs(np.sin(V))**(1/self.par_ellipsoide[3])

        else:

            center = np.array([0, 0, 0])  # x0, y0, z0
            a, b, c = 8, 8, 2                   # ellipse axes lengths
            self.par_ellipsoide = np.array([a,b,c])
            # theta = np.radians(0)        # rotation angle around Z axis
            # u = np.linspace(0, 2 * np.pi, 100)
            # v = np.linspace(0, np.pi, 50)

            # x = a * np.outer(np.cos(u), np.sin(v))
            # y = b * np.outer(np.sin(u), np.sin(v))
            # z = c * np.outer(np.ones_like(u), np.cos(v))

        self.dt=0.1
        self.evader_speed = np.array([19])
        self.pursuers_speed = np.array([20,40,30,21,32])
        # alpha = pursuers_speed/evader_speed
        self.r = np.array([30,15,20,50,25])
        #min 5cm/s max 100cm/s
        #ruido posicao 0.1
        #ruido velocidade metade da velocidade
        self.failure_rate = 0.0
        self.mode=1
        self.x0=[20.1, 50.1 , 100.0]

        
    
        

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()