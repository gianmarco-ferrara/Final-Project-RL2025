import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # ====================================================
    # 1. SETUP VARIABILI E PATH
    # ====================================================
    fra2mo_pkg = get_package_share_directory('ros2_fra2mo')
    iiwa_description_pkg = get_package_share_directory('iiwa_description')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    # Path Risorse
    models_path = os.path.join(fra2mo_pkg, 'models')
    world_file = os.path.join(fra2mo_pkg, "worlds", "warehouse.sdf")
    map_file = os.path.join(fra2mo_pkg, 'maps', 'map.yaml') 
    nav2_params = os.path.join(fra2mo_pkg, 'config', 'explore.yaml')
    iiwa_base_frame_file = os.path.join(fra2mo_pkg, 'config', 'base_frame_warehouse.yaml')

    # Variabile ambiente per modelli Gazebo
    env_var = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", 
        value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # ====================================================
    # 2. AVVIO GAZEBO SIM
    # ====================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [world_file, ' -r', ' -v 4']}.items()
    )

    # ====================================================
    # 3. FRA2MO (Mobile Robot)
    # ====================================================
    fra2mo_xacro = os.path.join(fra2mo_pkg, "urdf", "fra2mo.urdf.xacro")
    fra2mo_desc_val = ParameterValue(Command(['xacro ', fra2mo_xacro]), value_type=str)

    # RSP nel namespace globale
    fra2mo_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': fra2mo_desc_val, 'use_sim_time': True}]
    )

    # Spawn Fra2mo
    spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', 
                   '-name', 'fra2mo',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # ====================================================
    # 4. IIWA (Manipulator)
    # ====================================================
    iiwa_ns = "iiwa" # Stringa pulita
    iiwa_xacro_file = os.path.join(iiwa_description_pkg, 'config', 'iiwa.config.xacro')
    
    iiwa_command = Command([
        'xacro ', iiwa_xacro_file,
        ' prefix:=', 'iiwa_',                
        ' use_sim:=', 'true',
        ' use_fake_hardware:=', 'false',     
        # MODIFICA: Passiamo "iiwa" pulito. Lo Xacro corretto gestirà il resto.
        ' namespace:=', iiwa_ns,
        ' command_interface:=', 'position',
        ' base_frame_file:=', iiwa_base_frame_file 
    ])
    iiwa_desc_val = ParameterValue(iiwa_command, value_type=str)

    # RSP nel namespace 'iiwa'
    iiwa_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=iiwa_ns,
        parameters=[{'robot_description': iiwa_desc_val, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn IIWA
    spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/iiwa/robot_description', 
                   '-name', 'iiwa',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0', 
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output='screen'
    )

    # Controller Spawners
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/iiwa/controller_manager"],
    )
    load_arm_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["iiwa_arm_controller", "--controller-manager", "/iiwa/controller_manager"],
    )

    # Ritarda i controller
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_iiwa,
            on_exit=[load_jsb, load_arm_ctrl]
        )
    )

    # ====================================================
    # 5. BRIDGE & EXTRAS
    # ====================================================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/iiwa/detach@std_msgs/msg/Empty]ignition.msgs.Empty' 
        ],
        remappings=[
            ('/model/fra2mo/tf', '/tf'),
        ],
        output='screen'
    )

    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}]
    )

    nav2_bringup = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': 'true'
            }.items()
        )]
    )

    return LaunchDescription([
        env_var,
        gazebo,
        fra2mo_rsp,
        iiwa_rsp,
        spawn_fra2mo,
        spawn_iiwa,
        bridge,
        odom_tf,
        delay_controllers, 
        nav2_bringup       
    ])