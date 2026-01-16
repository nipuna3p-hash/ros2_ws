import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        
        # 1. Subscribe to Camera
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
            
        # 2. Subscribe to Joint States
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data)

        # 3. Publisher
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10)
            
        self.bridge = CvBridge()
        
        # State variables
        self.current_joint_1 = 0.0
        self.current_joint_2 = 0.0
        self.robot_initialized = False
        
        # 4. SEND HOME POSE IMMEDIATELY
        # We start a timer to send the home pose once connected
        self.home_timer = self.create_timer(1.0, self.move_to_home)
        self.is_homed = False

    def move_to_home(self):
        """Moves the robot to a bent 'ready' position."""
        if self.is_homed:
            self.home_timer.cancel() # Stop checking once done
            return

        self.get_logger().info("Moving to Home Pose (Bent Arm)...")
        
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        point = JointTrajectoryPoint()
        # POSE: Turn slightly, Bend Shoulder (-0.5), Bend Elbow (0.5), Bend Wrist (0.5)
        # This makes it look like a desk lamp looking down/forward
        point.positions = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
        point.time_from_start.sec = 2 # Take 2 seconds to move there safely
        
        traj.points.append(point)
        self.publisher_.publish(traj)
        
        # We assume after 2 seconds it's there
        self.is_homed = True

    def joint_state_callback(self, msg):
        if not self.robot_initialized:
            self.robot_initialized = True
        try:
            if "joint1" in msg.name:
                self.current_joint_1 = msg.position[msg.name.index("joint1")]
            if "joint2" in msg.name:
                self.current_joint_2 = msg.position[msg.name.index("joint2")]
        except ValueError:
            pass

    def image_callback(self, msg):
        # Don't track until we have finished homing!
        if not self.is_homed: 
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Blue Color Range
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
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                
                # Control Logic
                error_x = 400 - cx
                error_y = 300 - cy 
                k_p = 0.001 
                
                # Calculate Targets
                target_j1 = self.current_joint_1 + (error_x * k_p)
                # Note: We use '-' for J2 to look UP when object is HIGH (low pixel Y)
                target_j2 = self.current_joint_2 - (error_y * k_p)
                
                if abs(error_x) > 20 or abs(error_y) > 20:
                    self.move_robot(target_j1, target_j2)

        cv2.imshow("Robot Camera", cv_image)
        cv2.waitKey(1)

    def move_robot(self, j1, j2):
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        point = JointTrajectoryPoint()
        # Keep the elbow and wrist bent (0.5) so it maintains shape while tracking
        point.positions = [j1, j2, 0.5, 0.0, 0.5, 0.0]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000 
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