import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
import cv2
import numpy as np

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

class MoveItHelper:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, MoveGroup, 'move_action')
        self.node.get_logger().info("Waiting for MoveIt Action Server...")
        self.client.wait_for_server()
        self.node.get_logger().info("MoveIt Server Connected!")

    def move_to_joints(self, joint_names, target_values, timestamp):
        goal_msg = MoveGroup.Goal()
        
        # 1. Header (Using borrowed timestamp)
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.header.stamp = timestamp
        goal_msg.request.group_name = "arm"
        
        # 2. PLANNER SETTINGS (Back to OMPL)
        # We removed "pilz" and "PTP". Default is OMPL.
        # OMPL is much more robust against Gazebo timing issues.
        goal_msg.request.allowed_planning_time = 5.0 
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.request.num_planning_attempts = 10
        
        # 3. Constraints
        goal_constraints = Constraints()
        for name, val in zip(joint_names, target_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        self.node.get_logger().info(f"Sending OMPL Request... (Time: {timestamp.sec})")
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("MoveIt REJECTED the goal.")
            self.node.ready_for_next_move = True
            return
        
        self.node.get_logger().info("Goal Accepted. Executing...")
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val
        
        if error_code == 1:
            self.node.get_logger().info("Movement Complete!")
        elif error_code == -4: # CONTROL_FAILED
            self.node.get_logger().warn("Controller Error (Timing?). Retrying...")
        else:
            self.node.get_logger().error(f"Movement Failed: {error_code}")
            
        self.node.ready_for_next_move = True 

class VisualPlannerNode(Node):
    def __init__(self):
        super().__init__('visual_planner_node', 
                        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.bridge = CvBridge()
        self.moveit = MoveItHelper(self)
        self.joints = {}
        self.ready_for_next_move = True 
        self.latest_image = None
        self.latest_timestamp = None
        
        self.create_subscription(Image, '/camera1/image_raw', self.img_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        self.create_timer(1.0, self.planning_loop)
        
        self.TARGET_AREA = 30000
        self.LIMITS = { "joint1": (-3.0, 3.0), "joint2": (-1.5, 1.5), "joint3": (-1.5, 1.5) }

    def joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.joints[name] = msg.position[i]
        self.latest_timestamp = msg.header.stamp 

    def img_cb(self, msg):
        self.latest_image = msg 

    def planning_loop(self):
        if not self.ready_for_next_move or self.latest_image is None or not self.joints or self.latest_timestamp is None:
            return

        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([90, 50, 50]), np.array([140, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            M = cv2.moments(c)
            
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                error_x = 400 - cx
                error_y = 300 - cy
                error_area = self.TARGET_AREA - area
                
                if abs(error_x) < 50 and abs(error_y) < 50 and abs(error_area) < 5000:
                    self.get_logger().info("Target Aligned. Holding.")
                    return

                # Calculate Step
                step_j1 = error_x * 0.001
                step_j2 = -(error_y * 0.001)
                step_j3 = -(error_area * 0.00005)
                step_j2 = step_j2 - (step_j3 * 0.5)

                target_j1 = self.clamp(self.joints["joint1"] + step_j1, *self.LIMITS["joint1"])
                target_j2 = self.clamp(self.joints["joint2"] + step_j2, *self.LIMITS["joint2"])
                target_j3 = self.clamp(self.joints["joint3"] + step_j3, *self.LIMITS["joint3"])
                
                self.get_logger().info(f"Planning OMPL move...")
                self.ready_for_next_move = False 
                
                self.moveit.move_to_joints(
                    ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                    [target_j1, target_j2, target_j3, 0.0, 0.5, self.joints["joint6"]],
                    self.latest_timestamp 
                )

        cv2.imshow("Planner View", cv_img)
        cv2.waitKey(1)

    def clamp(self, val, min_v, max_v):
        return max(min_v, min(val, max_v))

def main(args=None):
    rclpy.init(args=args)
    node = VisualPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()