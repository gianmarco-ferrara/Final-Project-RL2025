import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # ====================================================
    # 1. PATHS DEFINITION
    # ====================================================
    # Packages and launch files
    fra2mo_pkg = get_package_share_directory('ros2_fra2mo')
    iiwa_description_pkg = get_package_share_directory('iiwa_description')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    # Gazebo custom world and navigation
    world_file = os.path.join(fra2mo_pkg, "worlds", "warehouse.sdf")
    models_path = os.path.join(fra2mo_pkg, 'models')
    map_file = os.path.join(fra2mo_pkg, 'maps', 'map.yaml') 
    nav2_params = os.path.join(fra2mo_pkg, 'config', 'navigation.yaml')
    rviz_config_file = os.path.join(fra2mo_pkg, 'rviz_conf', 'navigation.rviz')
    
    # IIWA robots
    iiwa_initial_positions_file = os.path.join(fra2mo_pkg, 'config', 'initial_positions.yaml') # initial configuration (same for both manipulators)
    iiwa_xacro_file = os.path.join(iiwa_description_pkg, 'config', 'iiwa.config.xacro')
    
    iiwa_base_frame_file = os.path.join(fra2mo_pkg, 'config', 'base_frame_warehouse.yaml')
    iiwa_controllers_file = 'iiwa_controllers.yaml'
    
    iiwa_2_base_frame_file = os.path.join(fra2mo_pkg, 'config', 'base_frame_shelf.yaml')
    iiwa_2_controllers_file = 'iiwa_2_controllers.yaml'
    

    # Gazebo environment variable (for meshes)
    env_var = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", 
        value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # ====================================================
    # 2. GAZEBO SIMULATION
    # ====================================================
    # Lancia Gazebo Sim con il mondo specificato
    # -r: run immediately (non in pausa)
    # -v 4: verbosity level (log dettagliati)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [world_file, ' -r', ' -v 4']}.items()
    )

    # ====================================================
    # 3. FRA2MO
    # ====================================================
    # Elaborazione del file XACRO per ottenere l'URDF del robot mobile
    fra2mo_xacro = os.path.join(fra2mo_pkg, "urdf", "fra2mo.urdf.xacro")
    fra2mo_desc_val = ParameterValue(Command(['xacro ', fra2mo_xacro]), value_type=str)

    # Pubblica lo stato del robot (TF statiche dei link) basandosi sull'URDF
    fra2mo_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': fra2mo_desc_val, 'use_sim_time': True}]
    )

    # Spawna fra2mo in Gazebo
    spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', 
                   '-name', 'fra2mo',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # Nodo custom per la pubblicazione della trasformata odom
    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{'use_sim_time': True}]
    )

    # TF Statica: Collega l'origine del mondo Gazebo (world) con la mappa di navigazione (map)
    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ====================================================
    # 4. IIWA 1 (Loading Area)
    # ====================================================
    iiwa_ns = "iiwa"
    
    # Comando Xacro complesso per configurare il braccio con i parametri specifici
    iiwa_command = Command([
        'xacro ', iiwa_xacro_file,
        ' prefix:=', 'iiwa_',                # Prefisso per i link/joint
        ' use_sim:=', 'true',
        ' use_fake_hardware:=', 'false',   
        ' namespace:=', iiwa_ns,
        ' command_interface:=', 'position',
        ' base_frame_file:=', iiwa_base_frame_file,
        ' initial_positions_file:=', iiwa_initial_positions_file,
        ' controllers_file:=', iiwa_controllers_file, 
        ' runtime_config_package:=', 'ros2_fra2mo'
    ])
    iiwa_desc_val = ParameterValue(iiwa_command, value_type=str)

    # Robot State Publisher
    iiwa_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=iiwa_ns,
        parameters=[{'robot_description': iiwa_desc_val, 'use_sim_time': True}],
        output='screen'
    )

    # Spawna il braccio 1
    spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/iiwa/robot_description', 
                   '-name', 'iiwa',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0', 
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output='screen'
    )

    # Caricamento controller: Joint State Broadcaster (legge giunti)
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["iiwa_joint_state_broadcaster", "--controller-manager", "/iiwa/controller_manager"],
    )
    # Caricamento controller: Arm Controller (muove il braccio)
    load_arm_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["iiwa_1_arm_controller", "--controller-manager", "/iiwa/controller_manager"],
    )

    # Event Handler: Attende che lo spawn del robot sia finito prima di lanciare i controller
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_iiwa,
            on_exit=[load_jsb, load_arm_ctrl]
        )
    )

    # ====================================================
    # 4-BIS. IIWA 2 (Unloading Area)
    # ====================================================
    iiwa_2_ns = "iiwa_2"
    
    iiwa_2_command = Command([
        'xacro ', iiwa_xacro_file,
        ' prefix:=', 'iiwa_2_',               # Prefisso diverso per evitare conflitti TF
        ' use_sim:=', 'true',
        ' use_fake_hardware:=', 'false',     
        ' namespace:=', iiwa_2_ns,
        ' command_interface:=', 'position',
        ' base_frame_file:=', iiwa_2_base_frame_file,
        ' initial_positions_file:=', iiwa_initial_positions_file,
        ' controllers_file:=', iiwa_2_controllers_file, 
        ' runtime_config_package:=', 'ros2_fra2mo'
    ])
    iiwa_2_desc_val = ParameterValue(iiwa_2_command, value_type=str)

    iiwa_2_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=iiwa_2_ns,
        parameters=[{'robot_description': iiwa_2_desc_val, 'use_sim_time': True}],
        output='screen'
    )

    spawn_iiwa_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/iiwa_2/robot_description', 
                   '-name', 'iiwa_2',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0', 
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output='screen'
    )

    load_jsb_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_2_broadcaster", "--controller-manager", "/iiwa_2/controller_manager"],
    )
    load_arm_ctrl_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["iiwa_2_arm_controller", "--controller-manager", "/iiwa_2/controller_manager"],
    )

    delay_controllers_2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_iiwa_2,
            on_exit=[load_jsb_2, load_arm_ctrl_2]
        )
    )

    # Raggruppiamo i nodi del secondo robot
    iiwa_2_group = [
        iiwa_2_rsp,
        spawn_iiwa_2,
        delay_controllers_2
    ]

    # TimerAction: Ritarda l'avvio del secondo robot di 5 secondi
    # Utile per non sovraccaricare Gazebo all'avvio simultaneo di troppi modelli
    delayed_iiwa_2_launch = TimerAction(
        period=5.0, 
        actions=iiwa_2_group
    )

    # ====================================================
    # 5. ROS-GAZEBO BRIDGE & NAVIGAZIONE
    # ====================================================
    # Il Bridge mappa i topic di Gazebo su topic ROS 2
    # @ for bidirectional bridges, [ for unidirectional bridges (from Gazebo to ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Cmd Vel e Odometria per Fra2mo
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            
            # Sensori Fra2mo
            '/model/fra2mo/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            
            # Servizio Teletrasporto
            '/world/warehouse/set_pose@ros_gz_interfaces/srv/SetEntityPose',

            # Topic Gripper (DetachableJoint) - Attach/Detach
            # IIWA 1
            '/iiwa/picker/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa/picker/detach@std_msgs/msg/Empty@ignition.msgs.Empty',

            # fra2mo
            '/fra2mo/picker/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/fra2mo/picker/detach@std_msgs/msg/Empty@ignition.msgs.Empty', 

            # IIWA 2
            '/iiwa_2/picker/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa_2/picker/detach@std_msgs/msg/Empty@ignition.msgs.Empty',
        ],
        remappings=[
            ('/model/fra2mo/joint_state', '/joint_states'),
            ('/camera', '/fra2mo/camera/image_raw'),
            ('/camera_info', '/fra2mo/camera/camera_info')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Avvio dello Stack di Navigazione (Nav2) con un ritardo per dare tempo al bridge e gazebo
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

    # ====================================================
    # 6. ARUCO DETECTORS
    # ====================================================
    
    # Detector 1: Cerca il Tag 115 (Vicino al primo IIWA)
    aruco_params_1 = {
        'image_is_rectified': True,
        'marker_size': 0.1,          
        'marker_id': 115,           
        'reference_frame': 'base_footprint', 
        'camera_frame': 'camera_link_optical',
        'marker_frame': 'aruco_marker_frame'
    }

    aruco_node_1 = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[aruco_params_1],
        remappings=[
            ('/camera_info', '/fra2mo/camera/camera_info'),
            ('/image', '/fra2mo/camera/image_raw'),
        ],
        output='screen'
    )

    # Detector 2: Cerca il Tag 201 (Vicino al secondo IIWA)
    aruco_params_2 = {
        'image_is_rectified': True,
        'marker_size': 0.1,
        'marker_id': 201,
        'reference_frame': 'base_footprint',
        'camera_frame': 'camera_link_optical',
        'marker_frame': 'aruco_marker_frame_2',
    }

    # Nota: Usiamo un namespace 'aruco_second' per evitare conflitti di nome nodo con il primo detector
    aruco_node_2 = Node(
        package='aruco_ros',
        executable='single',
        namespace='aruco_second', 
        name='aruco_node',         
        parameters=[aruco_params_2],
        remappings=[
            ('/camera_info', '/fra2mo/camera/camera_info'),
            ('/image', '/fra2mo/camera/image_raw'),
        ],
        output='screen'
    )
    
    # ====================================================
    # 7. RVIZ
    # ====================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Ritorna la descrizione completa del lancio
    return LaunchDescription([
        env_var,
        gazebo,
        bridge,
        fra2mo_rsp,
        spawn_fra2mo,
        odom_tf,
        world_to_map_tf,
        nav2_bringup,

        iiwa_rsp,
        spawn_iiwa,
        delay_controllers,

        delayed_iiwa_2_launch,

        aruco_node_1,
        aruco_node_2,
        rviz_node
    ])
