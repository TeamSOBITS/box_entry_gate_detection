import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    auto_configure = LaunchConfiguration('auto_configure')
    auto_activate = LaunchConfiguration('auto_activate')

    box_detection_node = LifecycleNode(
        package='box_entry_gate_detection',
        executable='box_detect',
        name='box_detection_node',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
            'sub_point_topic_name': launch.substitutions.LaunchConfiguration('sub_point_topic_name'),
            'base_frame_name': launch.substitutions.LaunchConfiguration('base_frame_name'),
            'depth_range_min_x': launch.substitutions.LaunchConfiguration('depth_range_min_x'),
            'depth_range_max_x': launch.substitutions.LaunchConfiguration('depth_range_max_x'),
            'depth_range_min_z': launch.substitutions.LaunchConfiguration('depth_range_min_z'),
            'depth_range_max_z': launch.substitutions.LaunchConfiguration('depth_range_max_z'),
            'cluster_ss': launch.substitutions.LaunchConfiguration('cluster_ss'),
            'shift_x': launch.substitutions.LaunchConfiguration('shift_x'),
            'shift_y': launch.substitutions.LaunchConfiguration('shift_y'),
            'shift_z': launch.substitutions.LaunchConfiguration('shift_z')
        }]
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(box_detection_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(auto_configure)
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=box_detection_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] box_detection_node is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(box_detection_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(auto_activate)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument('auto_configure', default_value='true', description='Configure lifecycle node on launch'),
        DeclareLaunchArgument('auto_activate', default_value='true', description='Activate lifecycle node after configure'),
        DeclareLaunchArgument('sub_point_topic_name', default_value='head_camera/depth/color/points"', description='Topic name for point cloud'),
        DeclareLaunchArgument('base_frame_name', default_value='base_footprint', description='Base frame name'),
        DeclareLaunchArgument('depth_range_min_x', default_value='0.0', description='Minimum depth in X direction'),
        DeclareLaunchArgument('depth_range_max_x', default_value='1.5', description='Maximum depth in X direction'),
        DeclareLaunchArgument('depth_range_min_z', default_value='0.1', description='Minimum depth in Z direction'),
        DeclareLaunchArgument('depth_range_max_z', default_value='1.5', description='Maximum depth in Z direction'),
        DeclareLaunchArgument('cluster_ss', default_value='0.05', description='Cluster resolution'),
        DeclareLaunchArgument('shift_x', default_value='0.0', description='Shift in X direction'),
        DeclareLaunchArgument('shift_y', default_value='0.0', description='Shift in Y direction'),
        DeclareLaunchArgument('shift_z', default_value='0.0', description='Shift in Z direction'),

        box_detection_node,
        configure_event,
        activate_event,
    ])
