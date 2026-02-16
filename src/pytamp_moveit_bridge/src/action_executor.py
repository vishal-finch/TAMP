#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from pytamp_moveit_bridge.msg import TaskAction
# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
        return True
    else:
        logger.error("Planning failed")
        return False

    time.sleep(sleep_time)

class ActionExecutor(Node):

    def __init__(self):
        super().__init__('action_executor')
        
        # Subscriber for Task Plans
        self.subscription = self.create_subscription(
            TaskAction,
            '/task_plan',
            self.execute_task_callback,
            10)
        self.subscription

        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "panda_link0"
        
        # Instantiate MoveItPy
        self.panda = MoveItPy(node_name="moveit_py_executor")
        self.panda_arm = self.panda.get_planning_component("panda_arm")
        self.panda_hand = self.panda.get_planning_component("hand")
        self.logger = get_logger("action_executor")

        robot_model = self.panda.get_robot_model()
        self.robot_state = RobotState(robot_model)

        # Config parameters
        self.pre_grasp_offset = 0.1 # meters above target
        self.init_angle = -0.3825 # alignment angle

    def execute_task_callback(self, task_action):
        self.get_logger().info(f"Received Action: {task_action.action_type} for Object: {task_action.object_id}")
        
        obj_pose = task_action.target_pose
        
        if task_action.action_type == "PICK":
            self.execute_pick(obj_pose)
        elif task_action.action_type == "PLACE":
            self.execute_place(obj_pose)
        elif task_action.action_type == "NAVIGATE":
            self.move_arm_to_pose(obj_pose)
        else:
            self.get_logger().warn(f"Unknown action type: {task_action.action_type}")

    def move_arm_to_pose(self, pose_msg, offset_z=0.0):
        self.pose_goal.pose.position.x = pose_msg.position.x
        self.pose_goal.pose.position.y = pose_msg.position.y
        self.pose_goal.pose.position.z = pose_msg.position.z + offset_z
        
        # Orientation: Use the planner provided orientation or default?
        # For now, blindly trust the planner's orientation
        self.pose_goal.pose.orientation = pose_msg.orientation
        
        # Set Goal
        self.panda_arm.set_goal_state(pose_stamped_msg = self.pose_goal, pose_link="panda_link8")
        result = plan_and_execute(self.panda, self.panda_arm, self.logger, sleep_time=0.5)
        return result

    def gripper_action(self, action):
        self.panda_hand.set_start_state_to_current_state()

        if action == 'OPEN':
            joint_values = {"panda_finger_joint1": 0.035}
        elif action == 'CLOSE':
            joint_values = {"panda_finger_joint1": 0.001} # Nearly closed
        else:
            return False

        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state = self.robot_state,
            joint_model_group = self.panda.get_robot_model().get_joint_model_group("hand"),
        )        
        self.panda_hand.set_goal_state(motion_plan_constraints=[joint_constraint])
        plan_and_execute(self.panda, self.panda_hand, self.logger, sleep_time=1.0)

    def execute_pick(self, target_pose):
        self.get_logger().info("Executing PICK sequence...")
        
        # 1. Open Gripper
        self.gripper_action("OPEN")
        
        # 2. Pre-Grasp (Above)
        self.get_logger().info("Moving to Pre-Grasp...")
        if not self.move_arm_to_pose(target_pose, offset_z=self.pre_grasp_offset):
            return # Failed
            
        # 3. Approach (Down)
        self.get_logger().info("Approaching Target...")
        if not self.move_arm_to_pose(target_pose, offset_z=0.0): # Exact height or slight offset?
            return 
            
        # 4. Close Gripper
        self.get_logger().info("Closing Gripper...")
        self.gripper_action("CLOSE")
        
        # 5. Lift
        self.get_logger().info("Lifting...")
        self.move_arm_to_pose(target_pose, offset_z=self.pre_grasp_offset)

    def execute_place(self, target_pose):
        self.get_logger().info("Executing PLACE sequence...")
        
        # 1. Move to Pre-Place (Above)
        if not self.move_arm_to_pose(target_pose, offset_z=self.pre_grasp_offset):
            return

        # 2. Descent
        if not self.move_arm_to_pose(target_pose, offset_z=0.0):
            return
            
        # 3. Open Gripper
        self.gripper_action("OPEN")
        
        # 4. Retreat
        self.move_arm_to_pose(target_pose, offset_z=self.pre_grasp_offset)

def main(args=None):
    rclpy.init(args=args)
    
    executor = ActionExecutor()
    
    # Needs MultiThreadedExecutor for MoveIt callbacks if needed?
    # Simple spin might block if MoveIt uses callbacks, but MoveItPy is usually sync planning.
    # However, rclpy callbacks need to run.
    
    rclpy_executor = rclpy.executors.MultiThreadedExecutor()
    rclpy_executor.add_node(executor)
    
    executor_thread = threading.Thread(target=rclpy_executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
