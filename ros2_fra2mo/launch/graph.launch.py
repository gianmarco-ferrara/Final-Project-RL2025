import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # ====================================================
    # 1. SETUP PATHS
    # ====================================================
    fra2mo_pkg = get_package_share_directory('ros2_fra2mo')
    iiwa_description_pkg = get_package_share_directory('iiwa_description')
    
    world_file = os.path.join(fra2mo_pkg, "worlds", "warehouse.sdf")
    models_path = os.path.join(fra2mo_pkg, 'models')
    
    # Configurazione IIWA 1
    iiwa_base_frame_file = os.path.join(fra2mo_pkg, 'config', 'base_frame_warehouse.yaml')
    iiwa_initial_positions_file = os.path.join(fra2mo_pkg, 'config', 'initial_positions.yaml')
    iiwa_controllers_file = 'iiwa_controllers.yaml'

    # Variabili d'ambiente per Gazebo
    env_var = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", 
        value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # ====================================================
    # 2. GAZEBO (Senza Navigazione)
    # ====================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [world_file, ' -r', ' -v 4']}.items()
    )

    # ====================================================
    # 3. ROBOT STATE PUBLISHERS & SPAWNERS
    # ====================================================
    
    # --- FRA2MO ---
    fra2mo_xacro = os.path.join(fra2mo_pkg, "urdf", "fra2mo.urdf.xacro")
    fra2mo_desc_val = ParameterValue(Command(['xacro ', fra2mo_xacro]), value_type=str)

    fra2mo_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': fra2mo_desc_val, 'use_sim_time': True}]
    )

    spawn_fra2mo = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/robot_description', '-name', 'fra2mo', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    odom_tf = Node(
        package='ros2_fra2mo', executable='dynamic_tf_publisher',
        name='odom_tf', parameters=[{'use_sim_time': True}]
    )

    # --- IIWA 1 ---
    iiwa_ns = "iiwa"
    iiwa_xacro_file = os.path.join(iiwa_description_pkg, 'config', 'iiwa.config.xacro')
    iiwa_command = Command([
        'xacro ', iiwa_xacro_file, ' prefix:=', 'iiwa_', ' use_sim:=', 'true', ' use_fake_hardware:=', 'false',
        ' namespace:=', iiwa_ns, ' command_interface:=', 'position',
        ' base_frame_file:=', iiwa_base_frame_file, ' initial_positions_file:=', iiwa_initial_positions_file,
        ' controllers_file:=', iiwa_controllers_file, ' runtime_config_package:=', 'ros2_fra2mo'
    ])
    iiwa_desc_val = ParameterValue(iiwa_command, value_type=str)

    iiwa_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', namespace=iiwa_ns,
        parameters=[{'robot_description': iiwa_desc_val, 'use_sim_time': True}], output='screen'
    )

    spawn_iiwa = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/iiwa/robot_description', '-name', 'iiwa', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output='screen'
    )

    # Controller IIWA 1
    load_jsb = Node(package="controller_manager", executable="spawner", arguments=["iiwa_joint_state_broadcaster", "--controller-manager", "/iiwa/controller_manager"])
    load_arm_ctrl = Node(package="controller_manager", executable="spawner", arguments=["iiwa_1_arm_controller", "--controller-manager", "/iiwa/controller_manager"])
    
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_iiwa, on_exit=[load_jsb, load_arm_ctrl])
    )

    # --- IIWA 2 ---
    iiwa_2_ns = "iiwa_2"
    iiwa_2_base_frame_file = os.path.join(fra2mo_pkg, 'config', 'base_frame_shelf.yaml')
    iiwa_2_controllers_file = 'iiwa_2_controllers.yaml'
    
    iiwa_2_command = Command([
        'xacro ', iiwa_xacro_file, ' prefix:=', 'iiwa_2_', ' use_sim:=', 'true', ' use_fake_hardware:=', 'false',
        ' namespace:=', iiwa_2_ns, ' command_interface:=', 'position',
        ' base_frame_file:=', iiwa_2_base_frame_file, ' initial_positions_file:=', iiwa_initial_positions_file,
        ' controllers_file:=', iiwa_2_controllers_file, ' runtime_config_package:=', 'ros2_fra2mo'
    ])
    iiwa_2_desc_val = ParameterValue(iiwa_2_command, value_type=str)

    iiwa_2_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', namespace=iiwa_2_ns,
        parameters=[{'robot_description': iiwa_2_desc_val, 'use_sim_time': True}], output='screen'
    )

    spawn_iiwa_2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/iiwa_2/robot_description', '-name', 'iiwa_2', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output='screen'
    )

    load_jsb_2 = Node(package="controller_manager", executable="spawner", arguments=["joint_state_2_broadcaster", "--controller-manager", "/iiwa_2/controller_manager"])
    load_arm_ctrl_2 = Node(package="controller_manager", executable="spawner", arguments=["iiwa_2_arm_controller", "--controller-manager", "/iiwa_2/controller_manager"])

    delay_controllers_2 = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_iiwa_2, on_exit=[load_jsb_2, load_arm_ctrl_2])
    )
    
    delayed_iiwa_2_launch = TimerAction(period=2.0, actions=[iiwa_2_rsp, spawn_iiwa_2, delay_controllers_2])

    # ====================================================
    # 4. BRIDGE (Essenziale per i topic)
    # ====================================================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            # Topics Gripper
            '/iiwa/picker/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa/picker/detach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/fra2mo/picker/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/fra2mo/picker/detach@std_msgs/msg/Empty@ignition.msgs.Empty', 
            '/iiwa_2/picker/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa_2/picker/detach@std_msgs/msg/Empty@ignition.msgs.Empty',
        ],
        remappings=[
            ('/model/fra2mo/joint_state', '/joint_states'),
            ('/camera', '/fra2mo/camera/image_raw'),
            ('/camera_info', '/fra2mo/camera/camera_info')
        ],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # ====================================================
    # 5. I TUOI NODI (Mission & Controllers)
    # ====================================================
    
    # FRA2MO MISSION (Nome corretto)
    fra2mo_mission_node = Node(
        package='ros2_fra2mo',
        executable='fra2mo_mission.py', 
        name='fra2mo_mission_node', # Nome assegnato esplicitamente
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # IIWA 1 CONTROLLER (Nome corretto)
    iiwa_1_node = Node(
        package='ros2_fra2mo',
        executable='iiwa_coordinated.py',
        name='iiwa1_node',          # Nome modificato
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # IIWA 2 CONTROLLER (Nome corretto)
    iiwa_2_node = Node(
        package='ros2_fra2mo',
        executable='iiwa_2_coordinated.py',
        name='iiwa2_node',          # Nome modificato
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Avvio ritardato dei nodi utente (solo 5 secondi invece di 15 per fare prima)
    # Il ritardo serve solo per aspettare che Gazebo carichi, ma senza Nav2 è più leggero.
    user_nodes_group = TimerAction(
        period=5.0,
        actions=[
            fra2mo_mission_node, 
            iiwa_1_node, 
            iiwa_2_node
        ]
    )

    return LaunchDescription([
        env_var,
        gazebo,
        bridge,
        fra2mo_rsp,
        spawn_fra2mo,
        odom_tf,
        iiwa_rsp,
        spawn_iiwa,
        delay_controllers,
        delayed_iiwa_2_launch,
        # Qui manca Nav2 Bringup intenzionalmente
        user_nodes_group
    ])
