#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import sys
import os
# Add current directory to path so we can import scene_interface installed next to this script
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

from pytamp_moveit_bridge.msg import WorldState, TaskAction
# Assuming scene_interface.py is in the same directory now
import scene_interface 
from scene_interface import PyTAMPSceneInterface
from std_msgs.msg import String

# Placeholder for MCTS if pytamp is not installed
# In real usage: from pytamp.search import MCTS

class MCTSPlannerNode(Node):
    def __init__(self):
        super().__init__('mcts_planner')
        
        # Parameters
        self.declare_parameter('urdf_path', '/path/to/robot.urdf') # Should be set via launch
        
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        if urdf_path == '/path/to/robot.urdf':
             # Try to find default
             from ament_index_python.packages import get_package_share_directory
             import os
             try:
                 urdf_path = os.path.join(get_package_share_directory('panda_moveit_config'), 'config', 'panda.urdf.xacro')
                 # Note: Xacro needs ensuring... using URDF is safer for pykin if xacro not parsed.
                 # For now, let's assume we pass a valid path or handle xacro in scene_interface
             except Exception:
                 pass

        # Initialize Scene Interface (The Brain's Cortex)
        try:
            self.scene_interface = PyTAMPSceneInterface(urdf_path)
            self.mcts_available = True
        except Exception as e:
            self.get_logger().error(f"Failed to init Scene Interface: {e}")
            self.mcts_available = False

        # Subscriber to World State (The Eyes)
        self.world_sub = self.create_subscription(WorldState, '/world_state', self.world_callback, 10)
        
        # Publisher for Plan (The Output)
        self.plan_pub = self.create_publisher(TaskAction, '/task_plan', 10)
        
        # Trigger for Planning (e.g. from UI or internal state)
        self.goal_sub = self.create_subscription(String, '/planning_goal', self.goal_callback, 10)

        self.get_logger().info("MCTS Planner Node Initialized. Waiting for World State...")

    def world_callback(self, msg):
        # Update internal scene representation
        if self.mcts_available:
            self.scene_interface.update_scene(msg)
            # self.get_logger().info("Scene Updated")

    def goal_callback(self, msg):
        self.get_logger().info(f"Received Goal: {msg.data}")
        if not self.mcts_available:
            self.get_logger().error("MCTS not available")
            return
            
        # 1. Define Goal for MCTS
        # task_goal = ...
        
        # 2. Run MCTS
        self.get_logger().info("Running MCTS Search...")
        # plan = MCTS.search(self.scene_interface, task_goal)
        
        # Placeholder Plan for Integration Testing
        # In real MCTS, this list comes from the tree search
        
        # Mocking a plan: Pick class 'RedBox' if available
        target_obj = None
        for obj_id, pose in self.scene_interface.objects.items():
             target_obj = obj_id # Pick the first one
             break
        
        if target_obj:
            self.get_logger().info(f"Generated Plan: Pick {target_obj}")
            
            # Action 1: Navigate to Approach
            # Action 2: Pick
            
            action_msg = TaskAction()
            action_msg.action_type = "PICK"
            action_msg.object_id = target_obj
            action_msg.target_pose = self.scene_interface.objects[target_obj]
            
            self.plan_pub.publish(action_msg)
        else:
            self.get_logger().warn("No objects found to plan for.")

def main(args=None):
    rclpy.init(args=args)
    node = MCTSPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
