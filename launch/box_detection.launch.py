import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument('execute_default', default_value='false', description='Execute default'),
        DeclareLaunchArgument('sub_point_topic_name', default_value='/hsrb/head_rgbd_sensor/depth_registered/points', description='Topic name for point cloud'),
        DeclareLaunchArgument('base_frame_name', default_value='base_footprint', description='Base frame name'),
        DeclareLaunchArgument('depth_range_min_x', default_value='0.0', description='Minimum depth in X direction'),
        DeclareLaunchArgument('depth_range_max_x', default_value='1.5', description='Maximum depth in X direction'),
        DeclareLaunchArgument('depth_range_min_z', default_value='0.1', description='Minimum depth in Z direction'),
        DeclareLaunchArgument('depth_range_max_z', default_value='0.7', description='Maximum depth in Z direction'),
        DeclareLaunchArgument('cluster_ss', default_value='0.05', description='Cluster resolution'),
        DeclareLaunchArgument('shift_x', default_value='0.0', description='Shift in X direction'),
        DeclareLaunchArgument('shift_y', default_value='0.0', description='Shift in Y direction'),
        DeclareLaunchArgument('shift_z', default_value='0.0', description='Shift in Z direction'),

        # Launch box_detection_node
        Node(
            package='box_entry_gate_detection',
            executable='box_detect',
            name='box_detection_node',
            output='screen',
            parameters=[{
                'execute_default': launch.substitutions.LaunchConfiguration('execute_default'),
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
        ),
    ])