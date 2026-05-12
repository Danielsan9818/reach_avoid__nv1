import rclpy
from rclpy.node import Node
import numpy as np

# Incoming message types (from your existing network)
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import Position
from std_msgs.msg import Float32MultiArray, Float64, Float64MultiArray

# Outgoing message types (for Foxglove)
from visualization_msgs.msg import Marker, MarkerArray

class FoxgloveVisualizerNode(Node):
    def __init__(self):
        super().__init__('foxglove_visualizer_node')
        self.get_logger().info('Foxglove Visualizer Node started.')

        # Parameters (Match these to your main node)
        self.declare_parameter('pursuers', ['C26']) 
        self.declare_parameter('evader', 'C25')
        self.declare_parameter('capture_radius', 0.3) # 30 cm radius from your code

        self.pursuers = self.get_parameter('pursuers').value
        self.evader = self.get_parameter('evader').value
        self.capture_radius = self.get_parameter('capture_radius').value
        self.all_agents = self.pursuers + [self.evader]

        # Internal State Variables
        self.target_center = np.zeros(3)
        self.target_params = np.array([0.08, 0.08, 0.02, 0.0]) # a, b, c, p
        self.which_area = 1
        self.first_cp_recorded = False
        self.first_cp = np.zeros(3)

        self.agent_positions = {name: np.zeros(3) for name in self.all_agents}

        # ==========================================
        # SUBSCRIBERS (Listening to your main node)
        # ==========================================
        self.create_subscription(NamedPoseArray, '/poses', self.poses_callback, 10)
        self.create_subscription(Position, '/target_center', self.target_center_callback, 10)
        self.create_subscription(Float32MultiArray, '/target_parameters', self.target_params_callback, 10)
        self.create_subscription(Position, '/optimal_capture_point', self.pursuer_cp_callback, 10)
        
        # NOTE: You will need to add publishers for these two in your main node (see instructions below)
        self.create_subscription(Float64, '/metrics/actual_speed', self.actual_speed_callback, 10)
        self.create_subscription(Float64, '/metrics/estimated_speed', self.estimated_speed_callback, 10)

        # ==========================================
        # PUBLISHERS (Broadcasting to Foxglove)
        # ==========================================
        self.marker_pub = self.create_publisher(MarkerArray, '/foxglove/agents_and_ranges', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/foxglove/target_area', 10)
        self.first_cp_pub = self.create_publisher(Marker, '/foxglove/first_capture_point', 10)
        self.current_cp_pub = self.create_publisher(Marker, '/foxglove/current_capture_point', 10)
        
        self.payoff_pub = self.create_publisher(Float64, '/foxglove/payoff', 10)
        self.distances_pub = self.create_publisher(Float64MultiArray, '/foxglove/pursuer_distances', 10)
        self.speeds_pub = self.create_publisher(Float64MultiArray, '/foxglove/evader_speeds', 10)

        # Timer to run the visualizer at 10Hz
        self.timer = self.create_timer(0.1, self.publish_visuals)


    # --- Callbacks to update internal state ---
    def target_center_callback(self, msg):
        self.target_center = np.array([msg.x, msg.y, msg.z])

    def target_params_callback(self, msg):
        if len(msg.data) == 5:
            self.target_params = np.array(msg.data[0:4])
            self.which_area = int(msg.data[4])

    def pursuer_cp_callback(self, msg):
        cp = np.array([msg.x, msg.y, msg.z])
        
        # Record the very first capture point
        if not self.first_cp_recorded and np.linalg.norm(cp) > 0:
            self.first_cp = cp
            self.first_cp_recorded = True

        # Publish the constantly updating capture point
        marker = self.create_sphere_marker("capture_points", 301, cp, [0.08, 0.08, 0.08], [1.0, 1.0, 0.0, 1.0]) # Yellow
        self.current_cp_pub.publish(marker)

    def actual_speed_callback(self, msg):
        self.current_actual_speed = msg.data

    def estimated_speed_callback(self, msg):
        self.current_estimated_speed = msg.data

    def poses_callback(self, msg):
        # Update current positions from motion capture
        for pose in msg.poses:
            if pose.name in self.agent_positions:
                self.agent_positions[pose.name][0] = pose.pose.position.x
                self.agent_positions[pose.name][1] = pose.pose.position.y
                self.agent_positions[pose.name][2] = pose.pose.position.z


    # --- Main Visualization Loop ---
    def publish_visuals(self):
        timestamp = self.get_clock().now().to_msg()
        frame_id = "map"

        # 1. Publish Agents and Capture Ranges
        marker_array = MarkerArray()
        
        for i, agent_name in enumerate(self.all_agents):
            pos = self.agent_positions[agent_name]
            is_evader = (agent_name == self.evader)

            # Agent Marker (Cube)
            color = [1.0, 0.0, 0.0, 1.0] if is_evader else [0.0, 0.0, 1.0, 1.0] # Red evader, Blue pursuer
            agent_marker = self.create_cube_marker("agents", i, pos, [0.1, 0.1, 0.1], color)
            marker_array.markers.append(agent_marker)

            # Capture Range (Transparent Sphere for Pursuers)
            if not is_evader:
                dia = self.capture_radius * 2
                range_marker = self.create_sphere_marker("capture_ranges", i+100, pos, [dia, dia, dia], [0.0, 0.5, 1.0, 0.2])
                marker_array.markers.append(range_marker)
                
        self.marker_pub.publish(marker_array)

        # 2. Publish Target Area
        if self.which_area == 1: # Ellipsoid
            scale = [self.target_params[0]*2, self.target_params[1]*2, self.target_params[2]*2]
            target_marker = self.create_sphere_marker("target", 200, self.target_center, scale, [0.0, 1.0, 0.0, 0.4])
        else: # Box/Other
            scale = [self.target_params[0]*2, self.target_params[1]*2, self.target_params[2]*2]
            target_marker = self.create_cube_marker("target", 200, self.target_center, scale, [0.0, 1.0, 0.0, 0.4])
        self.target_marker_pub.publish(target_marker)

        # 3. Publish First Capture Point
        if self.first_cp_recorded:
            first_cp_marker = self.create_sphere_marker("capture_points", 300, self.first_cp, [0.05, 0.05, 0.05], [1.0, 1.0, 1.0, 1.0])
            self.first_cp_pub.publish(first_cp_marker)

        # 4. Calculate and Publish Plot Metrics
        evader_pos_global = self.agent_positions[self.evader]
        evader_pos_local = evader_pos_global - self.target_center

        # Calculate Distances
        distances = []
        for pur in self.pursuers:
            dist = np.linalg.norm(evader_pos_global - self.agent_positions[pur])
            distances.append(float(dist))
        
        dist_msg = Float64MultiArray()
        dist_msg.data = distances
        self.distances_pub.publish(dist_msg)

        # Calculate Payoff
        payoff_msg = Float64()
        if self.which_area == 1:
            payoff_msg.data = float((evader_pos_local[0]**2) / self.target_params[0]**2 + (evader_pos_local[1]**2) / self.target_params[1]**2 + (evader_pos_local[2]**2) / self.target_params[2]**2 - 1)
        elif self.which_area == 2:
            payoff_msg.data = float((np.abs(evader_pos_local[0] / self.target_params[0]))**self.target_params[3] + (np.abs(evader_pos_local[1] / self.target_params[1]))**self.target_params[3]  + (np.abs(evader_pos_local[2] / self.target_params[2]))**self.target_params[3]  - 1)
        self.payoff_pub.publish(payoff_msg)

    # --- Helper functions to keep code clean ---
    def create_cube_marker(self, ns, id, pos, scale, color):
        return self._build_marker(Marker.CUBE, ns, id, pos, scale, color)

    def create_sphere_marker(self, ns, id, pos, scale, color):
        return self._build_marker(Marker.SPHERE, ns, id, pos, scale, color)

    def _build_marker(self, m_type, ns, id, pos, scale, color):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = id
        m.type = m_type
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        m.scale.x, m.scale.y, m.scale.z = float(scale[0]), float(scale[1]), float(scale[2])
        m.color.r, m.color.g, m.color.b, m.color.a = float(color[0]), float(color[1]), float(color[2]), float(color[3])
        return m

def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()