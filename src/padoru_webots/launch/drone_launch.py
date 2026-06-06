import os
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    use_rtabmap = LaunchConfiguration('rtabmap', default=False)

    package_dir = get_package_share_directory('padoru_webots')
    robot_description_path = os.path.join(
        package_dir, 'resource', 'drone_padoru2.urdf')

    with open(robot_description_path, 'r') as desc_file:
        robot_description = desc_file.read()

    footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'world']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        arguments=[robot_description_path]
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'robot_description': robot_description}]
    # )

    optical_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='optical_frame_publisher',
        output='screen',
        arguments=['0', '0', '0',
                   '1.57', '3.14', '1.57', 'camera', 'optical_link']
    )

    rtabmap_launch_path = get_package_share_directory('rtabmap_launch')
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_path, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'frame_id': 'base_link',
            'rgb_topic': '/Mavic_2_PRO/camera/image_raw',
            'depth_topic': '/Mavic_2_PRO/range_finder/image',
            'camera_info_topic': '/Mavic_2_PRO/camera/camera_info',
            'approx_sync': 'True',
            'rviz': 'False',
            'visual_odometry': 'False',
            'odom_topic': '/odom',
            "queue_size": "20",
        }.items(),
        condition=IfCondition(use_rtabmap)
    )

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'webots_world', 'worlds', 'test.wbt')
    )

    drone_driver = WebotsController(
        robot_name='Mavic 2 PRO',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=drone_driver,
        nodes_to_start=[rtabmap]
    )

    return LaunchDescription([
        webots,
        drone_driver,
        footprint,
        robot_state_publisher,
        optical_frame_publisher,
        # joint_state_publisher,
        waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )
        )
    ])
