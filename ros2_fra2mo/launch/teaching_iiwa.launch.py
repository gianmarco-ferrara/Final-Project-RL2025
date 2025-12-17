import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. PERCORSI
    iiwa_pkg = FindPackageShare('iiwa_description')
    fra2mo_pkg = FindPackageShare('ros2_fra2mo')
    
    xacro_file = PathJoinSubstitution([iiwa_pkg, 'config', 'iiwa.config.xacro'])
    # FONDAMENTALE: Usiamo il file che posiziona il robot nel magazzino
    base_frame_file = PathJoinSubstitution([fra2mo_pkg, 'config', 'base_frame_warehouse.yaml'])
    rviz_file = PathJoinSubstitution([iiwa_pkg, 'rviz', 'iiwa.rviz'])

    # 2. GENERAZIONE URDF
    # Passiamo base_frame_file altrimenti xacro fallisce e il robot non si vede
    robot_desc = ParameterValue(Command([
        'xacro ', xacro_file,
        ' prefix:=', 'iiwa_',
        ' base_frame_file:=', base_frame_file,
        ' command_interface:=', 'position'
    ]), value_type=str)

    # 3. NODI
    
    # Robot State Publisher (Pubblica la geometria del robot)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # Namespace vuoto per semplicità di visualizzazione
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # GUI Slider (Per muovere i giunti)
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen'
    )

    # 4. TARGET (COORDINATE ASSOLUTE DEL MAGAZZINO)
    # Non servono calcoli relativi. Mettiamo i marker dove stanno gli oggetti nel SDF.
    
    # BOX BLU (X=4.25, Y=4.0, Z=0.6)
    tf_box = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['4.25', '4.0', '0.6', '0', '0', '0', 'world', 'target_box']
    )
    
    # FRA2MO (X=3.54, Y=2.93, Z=0.25)
    tf_fra2mo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['3.54', '2.93', '0.3', '0', '0', '0', 'world', 'target_fra2mo']
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node,
        tf_box,
        tf_fra2mo
    ])