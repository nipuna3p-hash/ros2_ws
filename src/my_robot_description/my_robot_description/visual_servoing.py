import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        
        # 1. Subscribe to Camera (Best Effort QoS)
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
            
        # 2. Subscribe to Joint States (Best Effort QoS)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data)

        # 3. Publisher for Arm Controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10)
            
        self.bridge = CvBridge()
        
        # Robot State Variables
        self.current_joint_1 = 0.0
        self.current_joint_2 = 0.0
        self.robot_initialized = False

    def joint_state_callback(self, msg):
        """Updates the robot's current joint angles."""
        if not self.robot_initialized:
            self.get_logger().info(f"Connected to robot! Joints found: {msg.name}")
            self.robot_initialized = True

        try:
            # Update Joint 1 (Base Rotation)
            if "joint1" in msg.name:
                idx1 = msg.name.index("joint1")
                self.current_joint_1 = msg.position[idx1]
            
            # Update Joint 2 (Shoulder Pitch)
            if "joint2" in msg.name:
                idx2 = msg.name.index("joint2")
                self.current_joint_2 = msg.position[idx2]
                
        except ValueError:
            pass

    def image_callback(self, msg):
        """Processes camera image and sends control commands."""
        if not self.robot_initialized:
            return # Wait until we know where the robot is

        # 1. Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 2. Define Blue Color Range
        # If detection fails, widen these numbers (e.g. 90-140)
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([140, 255, 255])
        
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # 3. Find Contours (Blobs)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Find the largest blue object
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            
            if M['m00'] > 0:
                # Calculate Center (cx, cy)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Draw Green Dot for visualization
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                
                # --- CONTROL LOGIC ---
                # Image Center is (400, 300) for an 800x600 image
                
                # X-Axis Error (Controls Joint 1)
                error_x = 400 - cx
                
                # Y-Axis Error (Controls Joint 2)
                # If cy < 300 (object is high), we need to look UP.
                error_y = 300 - cy 
                
                # Control Gain (Sensitivity)
                # Lower this if the robot shakes/oscillates
                k_p = 0.001 
                
                # Calculate NEW target positions
                target_j1 = self.current_joint_1 + (error_x * k_p)
                
                # Note: 'joint2' usually needs '-' to look up when error is positive
                target_j2 = self.current_joint_2 - (error_y * k_p)
                
                # Deadband: Only move if error is significant (>20 pixels)
                if abs(error_x) > 20 or abs(error_y) > 20:
                    # Log the action
                    # self.get_logger().info(f"Tracking... X_err: {error_x} Y_err: {error_y}")
                    
                    # Move the Robot
                    self.move_robot(target_j1, target_j2)

        # Show the Camera View
        cv2.imshow("Robot Camera", cv_image)
        cv2.waitKey(1)

    def move_robot(self, j1, j2):
        """Sends a trajectory command to the controller."""
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        point = JointTrajectoryPoint()
        # Update J1 and J2, keep others at 0.0
        point.positions = [j1, j2, 0.0, 0.0, 0.0, 0.0]
        
        # Time from start defines speed/smoothness
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000 # 0.2 seconds
        
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