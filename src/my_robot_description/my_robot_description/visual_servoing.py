import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        
        # Subscribe to Camera and Joints (Best Effort for speed)
        self.subscription = self.create_subscription(
            Image, '/camera1/image_raw', self.image_callback, qos_profile_sensor_data)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos_profile_sensor_data)

        # Publisher
        self.publisher_ = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
            
        self.bridge = CvBridge()
        
        # Robot State Dictionary
        self.joints = {
            "joint1": 0.0, "joint2": 0.0, "joint3": 0.0,
            "joint4": 0.0, "joint5": 0.0, "joint6": 0.0
        }
        self.robot_initialized = False
        
        # --- TUNING PARAMETERS ---
        self.TARGET_AREA = 30000  # Target size of the blue box
        self.AREA_TOLERANCE = 2000 # Acceptable size difference (+/-)
        
        # INCREASED TOLERANCE: Makes it easier to trigger "Tightening"
        self.ALIGN_TOLERANCE = 100 # Pixel error allowed from center

        # Start Homing Sequence
        self.home_timer = self.create_timer(1.0, self.move_to_home)
        self.is_homed = False
        self.tightening_active = False

    def move_to_home(self):
        """Moves robot to a 'Ready' pose (bent arm)."""
        if self.is_homed:
            self.home_timer.cancel()
            return
            
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        point = JointTrajectoryPoint()
        # Ready Pose: Bent like a desk lamp
        point.positions = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]
        point.time_from_start.sec = 3
        traj.points.append(point)
        self.publisher_.publish(traj)
        self.is_homed = True
        self.get_logger().info("Robot moving to Home Pose...")

    def joint_state_callback(self, msg):
        """Updates internal joint states."""
        if not self.robot_initialized:
            self.robot_initialized = True
        try:
            for i, name in enumerate(msg.name):
                if name in self.joints:
                    self.joints[name] = msg.position[i]
        except ValueError:
            pass

    def image_callback(self, msg):
        # Don't track while homing or tightening
        if not self.is_homed or self.tightening_active:
            return 

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Blue Color Detection (Adjust if needed)
        mask = cv2.inRange(hsv, np.array([90, 50, 50]), np.array([140, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            M = cv2.moments(c)
            
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # Draw Green Dot
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                
                # --- CONTROL ERRORS ---
                error_x = 400 - cx      # Center X
                error_y = 300 - cy      # Center Y
                error_area = self.TARGET_AREA - area # Depth
                
                # Gains
                k_pan = 0.001   # Left/Right
                k_tilt = 0.001  # Up/Down
                k_zoom = 0.00005 # Forward/Back

                # Calculate New Joint Positions
                next_j1 = self.joints["joint1"] + (error_x * k_pan)
                next_j2 = self.joints["joint2"] - (error_y * k_tilt)
                next_j3 = self.joints["joint3"] - (error_area * k_zoom)

                # --- LOGIC: ALIGN VS APPROACH ---
                is_aligned = abs(error_x) < self.ALIGN_TOLERANCE and abs(error_y) < self.ALIGN_TOLERANCE
                is_close = abs(error_area) < self.AREA_TOLERANCE
                
                # NEW STATUS: Now shows Y_Err too!
                status = f"Area: {int(area)} | X_Err: {error_x} | Y_Err: {error_y}"

                if is_aligned and is_close:
                    self.trigger_tightening()
                    # Force print a new line so it stands out
                    print(f"\n!!! TIGHTENING ACTIVATED !!! (Area: {int(area)})")
                elif is_aligned:
                    # If aligned, allowed to approach (move Elbow J3)
                    self.move_robot(next_j1, next_j2, next_j3)
                    status += " | Approaching..."
                    print(status, end='\r')
                else:
                    # If not aligned, only Align (Lock Elbow J3)
                    self.move_robot(next_j1, next_j2, self.joints["joint3"])
                    status += " | Aligning..."
                    print(status, end='\r')

        cv_image = cv2.resize(cv_image, (400, 300)) # Resize window to fit screen
        cv2.imshow("Robot View", cv_image)
        cv2.waitKey(1)

    def trigger_tightening(self):
        """Simulates screw tightening by spinning Joint 6."""
        self.tightening_active = True
        
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        point = JointTrajectoryPoint()
        
        # Spin Joint 6 by adding 6*PI (3 full rotations)
        current_j6 = self.joints["joint6"]
        spin_target = current_j6 + (3.14 * 6) 
        
        # Keep current arm position, just spin the tool
        point.positions = [
            self.joints["joint1"],
            self.joints["joint2"],
            self.joints["joint3"],
            self.joints["joint4"],
            self.joints["joint5"],
            spin_target 
        ]
        
        point.time_from_start.sec = 4
        traj.points.append(point)
        self.publisher_.publish(traj)
        
        # Reset flag after 5 seconds
        self.create_timer(5.0, self.reset_tightening)

    def reset_tightening(self):
        self.tightening_active = False
        print("\n--- Tightening Complete. Resuming Tracking. ---")

    def move_robot(self, j1, j2, j3):
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        point = JointTrajectoryPoint()
        
        # Keep Wrist (J5) bent to look down
        point.positions = [j1, j2, j3, 0.0, 0.5, self.joints["joint6"]]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000 # 0.2s speed
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