import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_path = get_package_share_directory('control_pkg')

    time_arg=DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Dosya yolları
    gz_launch_path = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    world_path = os.path.join(pkg_path, "worlds", "my_world.sdf")
    robot_description_file = os.path.join(get_package_share_directory('saye_description'), 'urdf', 'model.xacro')

    # Launch Argument
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=world_path, description="Path to SDF world file"
    )

    # Xacro Dosyası İşleme
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Gazebo launch dosyasını içe aktar
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': [LaunchConfiguration('world')]}.items()
    )

    # Gazebo'da robotu spawn et
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'saye',
            '-topic', 'robot_description',
        ],
        output='screen'
    )

    # Bridge Node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_node',
        arguments=[
            # '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            # '/model/saye/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # '/model/saye/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
            # '/model/saye/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            # '/model/saye/pose_static@geometry_msgs/msg/Pose[gz.msgs.Pose_V',
            # '/model/saye/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # '/world/my_world/model/saye/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # '/world/my_world/model/saye/link/camera_back_link/sensor/color/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/my_world/model/saye/link/camera_back_link/sensor/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/my_world/model/saye/link/camera_back_link/sensor/depth/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/world/my_world/model/saye/link/camera_front_link/sensor/color/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/my_world/model/saye/link/camera_front_link/sensor/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/my_world/model/saye/link/camera_front_link/sensor/depth/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/world/my_world/model/saye/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/my_world/model/saye/link/scan_back_link/sensor/scan_back/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/world/my_world/model/saye/link/scan_omni_link/sensor/scan_omni/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/world/my_world/model/saye/link/scan_omni_link/sensor/scan_omni/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/sensors/marker@visualization_msgs/msg/MarkerArray[gz.msgs.Marker'
        ],    
        remappings=[
            ('/model/saye/pose', '/pose'),
            ('/model/saye/pose_static', '/pose_static'),
            ('/model/saye/tf', '/tf'),
            ('/cmd_vel', '/cmd_vel'),
            ('/model/saye/odometry', '/odom'),
            ('/model/saye/OdometryWithCovariance', '/odom_covariance'),
            ('/world/my_world/model/saye/joint_state', '/joint_state'),
            ('/world/my_world/model/saye/link/camera_front_link/sensor/color/image', 'camera_front/image_raw'),
            ('/world/my_world/model/saye/link/camera_front_link/sensor/depth/depth_image', 'camera_front/depth_image'),
            ('/world/my_world/model/saye/link/camera_back_link/sensor/color/image', 'camera_back/image_raw'),
            ('/world/my_world/model/saye/link/camera_back_link/sensor/depth/depth_image', 'camera_back/depth_image'),
            ('/world/my_world/model/saye/link/scan_front_link/sensor/scan_front/scan', '/front_laser/scan'),
            ('/world/my_world/model/saye/link/scan_back_link/sensor/scan_back/scan', '/back_laser/scan'),
            ('/world/my_world/model/saye/link/scan_omni_link/sensor/scan_omni/scan', '/omni_laser/scan'),
            ('/world/my_world/model/saye/link/imu_link/sensor/imu/imu', '/imu')
        ],
        output='screen'
    )



    return LaunchDescription([
        time_arg,
        declare_world_arg,
        gazebo,
        spawn,
        robot_state_publisher,
        bridge
    ])
