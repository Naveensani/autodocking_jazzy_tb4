from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tb4_apriltag_dir = get_package_share_directory('tb4_apriltag_bringup_cpp')
    
    # Path to custom world file
    world_file = PathJoinSubstitution([
        tb4_apriltag_dir,
        'worlds',
        'depot_with_apriltag.sdf'
    ])

    # Path to our custom params file with docking config
    params_file = PathJoinSubstitution([
        tb4_apriltag_dir,
        'config',
        'nav2_params_with_docking.yaml'
    ])
    
    # Path to apriltag config file
    apriltag_config = PathJoinSubstitution([
        tb4_apriltag_dir,
        'config',
        'apriltag_params.yaml'
    ])
    
    # Set GZ_SIM_RESOURCE_PATH to include apriltag models
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        '/root/gazebo_apriltag/:' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    # Include the original TB4 simulation launch with custom world
    tb4_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb4_simulation_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'headless': 'False',
            'world': world_file,
            'params_file': params_file,  # Use our custom params!
        }.items()
    )
    
    # Add rectify node for image processing
    rectify_node = Node(
        package="image_proc",
        executable="rectify_node",
        remappings=[
            ("image", "/rgbd_camera/image"),
            ("camera_info", "/rgbd_camera/camera_info"),
            ("image_rect", "/rgbd_camera/image_rect")
        ]
    )
    
    # Add AprilTag detection node
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        parameters=[apriltag_config],
        remappings=[
            ("image_rect", "/rgbd_camera/image_rect"),
            ("camera_info", "/rgbd_camera/camera_info"),
        ],
        output="screen"
    )
    
    # Detected Dock Pose Publisher
    detected_dock_pose_publisher = Node(
        package='tb4_apriltag_bringup_cpp',
        executable='detected_dock_pose_publisher',
        name='detected_dock_pose_publisher',
        parameters=[{
            'parent_frame': 'oakd_rgb_camera_optical_frame',
            'child_frame': 'tag36h11:0',
            'publish_rate': 10.0
        }],
        output='screen'
    )

    # # Docking Server
    # docking_server1 = Node(
    #     package='opennav_docking',
    #     executable='opennav_docking',
    #     name='docking_server',
    #     parameters=[docking_config],
    #     output='screen'
    # )
    
    return LaunchDescription([
        gz_resource_path,
        tb4_simulation,
        rectify_node,
        apriltag_node,
        detected_dock_pose_publisher,
    ])