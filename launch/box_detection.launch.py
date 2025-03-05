import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument('execute_default', default_value='false', description='Execute default'),  # 'false' にする
        DeclareLaunchArgument('sub_point_topic_name', default_value='/hsrb/head_rgbd_sensor/depth/points', description='Topic name for point cloud'),
        DeclareLaunchArgument('base_frame_name', default_value='base_footprint', description='Base frame name'),
        DeclareLaunchArgument('depth_range_min_x', default_value='0.1', description='Minimum depth in X direction'),
        DeclareLaunchArgument('depth_range_max_x', default_value='1.2', description='Maximum depth in X direction'),
        DeclareLaunchArgument('depth_range_min_z', default_value='0.1', description='Minimum depth in Z direction'),
        DeclareLaunchArgument('depth_range_max_z', default_value='1.2', description='Maximum depth in Z direction'),
        DeclareLaunchArgument('cluster_ss', default_value='0.05', description='Cluster resolution'),
        DeclareLaunchArgument('shift_x', default_value='0.0', description='Shift in X direction'),
        DeclareLaunchArgument('shift_y', default_value='0.0', description='Shift in Y direction'),
        DeclareLaunchArgument('shift_z', default_value='0.2', description='Shift in Z direction'),

        # Launch box_detection_node
        Node(
            package='box_entry_gate_detection',
            executable='box_detect',
            name='box_detection_node',
            output='screen',
            parameters=[{
                'execute_default': False,  # bool のまま
                'sub_point_topic_name': '/hsrb/head_rgbd_sensor/depth/points',
                'base_frame_name': 'base_footprint',
                'depth_range_min_x': 0.1,
                'depth_range_max_x': 1.2,
                'depth_range_min_z': 0.1,
                'depth_range_max_z': 1.2,
                'cluster_ss': 0.05,
                'shift_x': 0.0,
                'shift_y': 0.0,
                'shift_z': 0.2
            }]
        ),
    ])
