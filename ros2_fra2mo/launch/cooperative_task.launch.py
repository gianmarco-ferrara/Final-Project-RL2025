import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # path of package 'ros2_fra2mo'
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')
    
    # ========================================================================
    # 1. CARICAMENTO AMBIENTE E ROBOT
    # ========================================================================
    # Include ed esegue il launch file 'fra2mo_iiwa.launch.py' che si occupa di:
    # - Aprire Gazebo
    # - Spawnare Fra2mo, IIWA 1 e IIWA 2
    # - Avviare Nav2 (mappe e navigazione)
    # - Avviare i nodi Aruco e il Bridge ROS-Gazebo
    env_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fra2mo, 'launch', 'fra2mo_iiwa.launch.py')
        )
    )

    # ========================================================================
    # 2. DEFINIZIONE NODI DI LOGICA (MISSION & CONTROLLERS)
    # ========================================================================
    
    # Nodo "Cervello" del Robot Mobile (Fra2mo)
    fra2mo_mission_node = Node(
        package='ros2_fra2mo',
        executable='fra2mo_mission.py', 
        name='fra2mo_mission_node',
        output='screen',
        parameters=[{'use_sim_time': True}] # Fondamentale per sincronizzarsi con Gazebo
    )

    # Nodo di controllo per il PRIMO braccio (IIWA 1)
    iiwa_1_node = Node(
        package='ros2_fra2mo',
        executable='iiwa_coordinated.py',
        name='iiwa1_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Nodo di controllo per il SECONDO braccio (IIWA 2)
    iiwa_2_node = Node(
        package='ros2_fra2mo',
        executable='iiwa_2_coordinated.py',
        name='iiwa2_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ========================================================================
    # 3. AVVIO RITARDATO DELLA MISSIONE (TIMER ACTION)
    # ========================================================================
    # I nodi di "intelligenza" (Python) vengono avviati con un ritardo di 15 secondi
    # rispetto al lancio dell'ambiente.
    # MOTIVO:
    # Gazebo, Nav2 e i controller dei robot ci mettono tempo ad inizializzarsi.
    # Se avviassimo subito la missione, i nodi Python fallirebbero perché
    # non troverebbero ancora i topic attivi o le action server pronti.
    mission_group = TimerAction(
        period=20.0,
        actions=[
            fra2mo_mission_node, 
            iiwa_1_node, 
            iiwa_2_node
        ]
    )

    # Restituisce la descrizione: Prima lancia l'ambiente, poi (dopo 15s) la logica.
    return LaunchDescription([
        env_launch,
        mission_group
    ])
