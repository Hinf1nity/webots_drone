import os
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, ComposableNodeContainer, SetRemap, SetParameter, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    # use_rtabmap = LaunchConfiguration('rtabmap', default=False)

    package_dir = get_package_share_directory('thaiquiri_webots')
    robot_description_path = os.path.join(
        package_dir, 'resource', 'thaiquiri.urdf')

    with open(robot_description_path, 'r') as desc_file:
        robot_description = desc_file.read()

    footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '1.5708', 'world', 'base_link']
    )

    camera_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_publisher',
        output='screen',
        arguments=['0', '0', '0', '1.5708', '0',
                   '1.5708', 'base_link', 'ZED_X_Mini_camera']
        # arguments=['0', '0', '0', '0', '0',
        #            '0', 'base_link', 'ZED_X_Mini_camera']
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

    # rtabmap_launch_path = get_package_share_directory('rtabmap_launch')
    # rtabmap = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(rtabmap_launch_path, 'launch', 'rtabmap.launch.py')
    #     ),
    #     launch_arguments={
    #         'rtabmap_args': '--delete_db_on_start',
    #         'frame_id': 'base_link',
    #         'rgb_topic': '/drone/left/image_raw',
    #         'depth_topic': '/drone/points2',
    #         'camera_info_topic': '/drone/camera_left/camera_info',
    #         'approx_sync': 'True',
    #         'rviz': 'False',
    #         'visual_odometry': 'False',
    #         'odom_topic': '/odom',
    #         "queue_size": "20",
    #     }.items(),
    #     condition=IfCondition(use_rtabmap)
    # )

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'webots_world',
                           'worlds', 'mine_thaiquiri.wbt')
    )

    drone_driver = WebotsController(
        robot_name='drone',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    fix_camera_info = Node(
        package='thaiquiri_webots',
        executable='fix_camera_info',
        name='fix_camera_info',
        output='screen',
        namespace='drone',
    )

    bgra_to_bgr_node = Node(
        package='thaiquiri_webots',
        executable='bgra_to_bgr',
        name='bgra_to_bgr_node',
        output='screen',
        namespace='drone',
    )

    stereo_proc_share = get_package_share_directory('stereo_image_proc')
    stereo_launch_path = os.path.join(
        stereo_proc_share, 'launch', 'stereo_image_proc.launch.py')
    stereo_proc_group = GroupAction(
        actions=[
            # Incluir el archivo de lanzamiento oficial con los argumentos de sub-namespace que espera
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(stereo_launch_path),
                launch_arguments={
                    'namespace': 'drone',
                    'approximate_sync': 'True',
                    'approximate_sync_tolerance_seconds': '0.05',
                }.items()
            )
        ]
    )
    return LaunchDescription([
        webots,
        drone_driver,
        footprint,
        camera_frame_publisher,
        robot_state_publisher,
        fix_camera_info,
        bgra_to_bgr_node,
        stereo_proc_group,
        # joint_state_publisher,
        # waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )
        )
    ])
