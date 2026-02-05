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
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.header.stamp = timestamp
        goal_msg.request.group_name = "arm"
        
        # OMPL Planner Settings (Robust)
        goal_msg.request.allowed_planning_time = 2.0 
        goal_msg.request.max_velocity_scaling_factor = 0.5 
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.request.num_planning_attempts = 5
        
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
        
        self.node.get_logger().info(f"Sending Request... (Time: {timestamp.sec})")
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("MoveIt REJECTED the goal.")
            self.node.ready_for_next_move = True
            return
        
        # self.node.get_logger().info("Goal Accepted. Moving...")
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val != 1:
            self.node.get_logger().error(f"Movement Failed: {result.error_code.val}")
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
        self.create_timer(0.5, self.planning_loop) 
        
        # --- TUNING ---
        self.TARGET_AREA = 30000 
        # Direction Multipliers (1 or -1)
        self.DIR_X = -1 
        self.DIR_Y = -1
        self.DIR_Z = -1

        # Strict Limits to prevent "Stuck" behavior
        # Joint 1: -3.0 to 3.0
        # Joint 2: -1.5 to 1.5
        # Joint 3: -1.5 to 1.5
        self.LIMITS = { 
            "joint1": (-3.0, 3.0), 
            "joint2": (-1.5, 1.5), 
            "joint3": (-1.5, 1.5) 
        }

    def joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.joints[name] = msg.position[i]
        self.latest_timestamp = msg.header.stamp 

    def img_cb(self, msg):
        self.latest_image = msg 

    def planning_loop(self):
        if not self.ready_for_next_move or self.latest_image is None or not self.joints:
            return

        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([90, 50, 50]), np.array([140, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw Target Crosshair (Red)
        cx_target, cy_target = 400, 300
        cv2.circle(cv_img, (cx_target, cy_target), 5, (0, 0, 255), -1)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            M = cv2.moments(c)
            
            if M['m00'] > 0:
                # Actual Object Center (Green)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(cv_img, (cx, cy), 10, (0, 255, 0), -1)
                
                # Calculate Errors
                error_x = cx_target - cx
                error_y = cy_target - cy
                error_area = self.TARGET_AREA - area
                
                # --- CHECK 1: ARE WE ALREADY THERE? ---
                # Tolerance: 40 pixels, Area diff: 5000
                if abs(error_x) < 40 and abs(error_y) < 40 and abs(error_area) < 5000:
                    cv2.putText(cv_img, "GOAL REACHED", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow("Robot Eye", cv_img)
                    cv2.waitKey(1)
                    return # Stop planning, just wait

                # --- CHECK 2: CALCULATE MOVES ---
                step_j1 = error_x * 0.001 * self.DIR_X
                step_j2 = error_y * 0.001 * self.DIR_Y
                step_j3 = 0.0
                
                # Only move forward if we are roughly pointing at it
                if abs(error_x) < 100 and abs(error_y) < 100:
                    step_j3 = error_area * 0.00005 * self.DIR_Z
                    step_j2 = step_j2 - (step_j3 * 0.5) # Compensation

                # --- CHECK 3: JOINT LIMITS ---
                next_j1 = self.joints["joint1"] + step_j1
                next_j2 = self.joints["joint2"] + step_j2
                next_j3 = self.joints["joint3"] + step_j3

                # Check if we hit a wall
                hit_limit = False
                if not (self.LIMITS["joint1"][0] < next_j1 < self.LIMITS["joint1"][1]):
                    cv2.putText(cv_img, "LIMIT J1", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    next_j1 = self.joints["joint1"] # Don't move this joint
                    hit_limit = True
                
                if not (self.LIMITS["joint2"][0] < next_j2 < self.LIMITS["joint2"][1]):
                    cv2.putText(cv_img, "LIMIT J2", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    next_j2 = self.joints["joint2"]
                    hit_limit = True

                # Apply Clamps
                final_j1 = self.clamp(next_j1, *self.LIMITS["joint1"])
                final_j2 = self.clamp(next_j2, *self.LIMITS["joint2"])
                final_j3 = self.clamp(next_j3, *self.LIMITS["joint3"])

                # Execute
                self.get_logger().info(f"Plan: J1={final_j1:.2f} J2={final_j2:.2f} J3={final_j3:.2f}")
                self.ready_for_next_move = False 
                
                self.moveit.move_to_joints(
                    ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                    [final_j1, final_j2, final_j3, 0.0, 0.5, self.joints["joint6"]],
                    self.latest_timestamp 
                )

        cv2.imshow("Robot Eye", cv_img)
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