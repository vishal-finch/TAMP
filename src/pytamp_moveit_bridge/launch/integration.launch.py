from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    
    # Arguments
    mock_arg = DeclareLaunchArgument(
        'use_mock_camera',
        default_value='true',
        description='Whether to publish static TF for mock camera'
    )
    
    # Load MoveIt Configs
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(os.path.join(get_package_share_directory("panda_moveit_config"), "config", "controller_setting.yaml"))
        .to_moveit_configs()
    )
    
    # Path to existing launch file for YOLOv8 (if we want to launch it here)
    # yolo_pkg = get_package_share_directory('yolov8_obb')
    # ...
    
    # 1. Object State Manager
    state_manager = Node(
        package='pytamp_moveit_bridge',
        executable='object_state_manager.py',
        name='object_state_manager',
        parameters=[{'use_sim_time': LaunchConfiguration('use_mock_camera', default='false') == 'false'}],
        output='screen'
    )
    
    # 2. Planner Node
    planner = Node(
        package='pytamp_moveit_bridge',
        executable='planner_node.py',
        name='mcts_planner',
        output='screen',
        parameters=[
            {'urdf_path': '/home/user/Vishal_project/pickplace/yolo/moveit2_obb/src/panda_moveit_config/config/panda.urdf.xacro'}
        ]
    )
    
    # 3. Action Executor
    executor = Node(
        package='pytamp_moveit_bridge',
        executable='action_executor.py',
        name='action_executor',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments = ['0.5', '0.0', '1.0', '0.0', '1.0', '0.0', '0.0', 'panda_link0', 'camera_optical_frame'],
        condition=IfCondition(LaunchConfiguration('use_mock_camera'))
    )

    # 5. Image View (User requested)
    # Using rqt_image_view to see what the robot sees
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_viewer',
        arguments=['/camera/color/image_raw'], # Default topic hint
        output='screen'
    )

    # 6. YOLOv8 OBB Publisher (Real Eyes)
    yolo_node = Node(
        package='yolov8_obb',
        executable='yolov8_obb_publisher.py',
        name='yolov8_obb_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_mock_camera', default='false') == 'false'}],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_mock_camera'))
    )

    # 6b. Mock Perception (Fake Eyes for Testing)
    mock_node = Node(
        package='pytamp_moveit_bridge',
        executable='mock_perception.py',
        name='mock_perception',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mock_camera'))
    )

    # 7. Robust Clock Bridge (The Heartbeat)
    # Ensures time sync even if simulation launch misses it
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        mock_arg,
        static_tf,
        image_view,
        yolo_node,
        state_manager,
        planner,
        executor,
        clock_bridge
    ])
