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
    
    # FONDAMENTALE: Usiamo il file che posiziona il SECONDO robot vicino alla shelf
    base_frame_file = PathJoinSubstitution([fra2mo_pkg, 'config', 'base_frame_shelf.yaml'])
    
    rviz_file = PathJoinSubstitution([iiwa_pkg, 'rviz', 'iiwa.rviz'])

    # 2. GENERAZIONE URDF
    # Nota: prefix deve essere 'iiwa_2_' per corrispondere ai nomi dei giunti nel controller reale
    robot_desc = ParameterValue(Command([
        'xacro ', xacro_file,
        ' prefix:=', 'iiwa_2_',
        ' base_frame_file:=', base_frame_file,
        ' command_interface:=', 'position'
    ]), value_type=str)

    # 3. NODI
    
    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # GUI Slider (Per muovere i giunti di iiwa_2)
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

    # 4. TARGET VISUALI (COORDINATE STATICHE CALCOLATE)
    
    # TARGET PICK: Dove si troverà il pacco sopra Fra2mo
    # Calcolo: 
    # - Tag Aruco Shelf X ~= -4.4
    # - Distanza arresto servoing = 0.45 -> Fra2mo Base X ~= -3.95
    # - Fra2mo orientato verso Ovest (180 deg)
    # - Box offset locale (-0.08) -> In coordinate mondo (+X) -> -3.95 + 0.08 = -3.87
    # - Altezza Box (teleport) = 0.3
    tf_pick_fra2mo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-3.87', '-3.6', '0.3', '0', '0', '0', 'world', 'target_pick_fra2mo']
    )
    
    # TARGET PLACE: Mensola Centrale della Libreria
    # Da warehouse.sdf: Shelf Pose Y = -4.6, X = -4.1
    # Visual Middle Z = 1.0 (Surface) -> Mettiamo 1.05 per il centro del box
    tf_place_shelf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-4.1', '-4.6', '1.05', '0', '0', '0', 'world', 'target_place_shelf']
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node,
        tf_pick_fra2mo,
        tf_place_shelf
    ])
