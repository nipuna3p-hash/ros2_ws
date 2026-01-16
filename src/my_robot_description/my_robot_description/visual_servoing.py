import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        
        # 1. Subscribe to Camera
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
            
        # 2. Subscribe to Joint States (With DEBUG print)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # 3. Publisher
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10)
            
        self.bridge = CvBridge()
        self.current_joint_1 = 0.0
        
        # --- BYPASS: Force True to start immediately ---
        self.robot_initialized = True 
        self.debug_printed = False

    def joint_state_callback(self, msg):
        # DEBUG: Print the first message we receive to see what's wrong
        if not self.debug_printed:
            self.get_logger().info(f"DEBUG: Received joint states: {msg.name}")
            self.debug_printed = True

        try:
            if "joint1" in msg.name:
                idx = msg.name.index("joint1")
                self.current_joint_1 = msg.position[idx]
        except ValueError:
            pass

    def image_callback(self, msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Color Tuning
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([140, 255, 255])
        
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Draw Green Dot
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                
                # Control Logic
                error = 400 - cx
                
                if abs(error) > 20: # Deadband
                    k_p = 0.002
                    target_angle = self.current_joint_1 + (error * k_p)
                    
                    self.get_logger().info(f"Target: {target_angle:.2f} | Error: {error}")
                    self.move_robot(target_angle)
        
        cv2.imshow("Robot Camera", cv_image)
        cv2.waitKey(1)

    def move_robot(self, j1_angle):
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        point = JointTrajectoryPoint()
        point.positions = [j1_angle, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000 
        traj.points.append(point)
        self.publisher_.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()