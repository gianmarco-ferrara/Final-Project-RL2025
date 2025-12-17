#! /usr/bin/env python3

import rclpy
import time
import math
from rclpy.node import Node
# Messaggi standard per geometria e comandi vuoti
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from std_msgs.msg import Empty
# Nav2 API per inviare goal di navigazione standard
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# Librerie per gestire le trasformate (TF)
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
# Servizio Gazebo per il teletrasporto degli oggetti
from ros_gz_interfaces.srv import SetEntityPose

# =============================================================================
# CONFIGURAZIONE TOPIC E COSTANTI
# =============================================================================
# Topic per attivare/disattivare il link logico (gripper) tra robot e scatola
TOPIC_FRA2MO_ATTACH = '/fra2mo/picker/attach'
TOPIC_FRA2MO_DETACH = '/fra2mo/picker/detach'

# Topic di sincronizzazione con i bracci robotici (IIWA)
TOPIC_SYNC_READY_1     = '/coordination/fra2mo_ready'      # Fra2mo -> IIWA1: "Sono in posizione"
TOPIC_SYNC_DONE_1      = '/coordination/iiwa_done'         # IIWA1 -> Fra2mo: "Ho finito, puoi andare"
TOPIC_SYNC_READY_2     = '/coordination/fra2mo_at_shelf'   # Fra2mo -> IIWA2: "Sono allo scaffale"
TOPIC_SYNC_IIWA_PICKED = '/coordination/iiwa_2_has_picked' # IIWA2 -> Fra2mo: "Ho preso la scatola"
TOPIC_SYNC_DONE_2      = '/coordination/iiwa_2_done'       # IIWA2 -> Fra2mo: "Ciclo finito"

# Stati della Macchina a Stati Finiti (FSM) per il Visual Servoing
STATE_SEARCH   = 0  # Ruota per cercare il tag
STATE_ALIGN    = 1  # Ruota per centrare il tag
STATE_APPROACH = 2  # Avanza verso il tag
STATE_FINISHED = 3  # Target raggiunto

class VisualServoingNode(Node):
    """
    Nodo che gestisce:
    1. Il Visual Servoing (avvicinamento di precisione con Aruco)
    2. La logica di "Missione" (Gripper, Teletrasporto)
    3. La sincronizzazione con i robot esterni
    """
    def __init__(self):
        super().__init__('fra2mo_mission_node')
        
        # --- SUBSCRIBERS (Visione) ---
        # Ascoltiamo due topic diversi perché usiamo due istanze di aruco_ros
        self.aruco_sub_1 = self.create_subscription(PoseStamped, '/aruco_single/pose', self.aruco_1_callback, 10)
        self.aruco_sub_2 = self.create_subscription(PoseStamped, '/aruco_second/aruco_node/pose', self.aruco_2_callback, 10)

        # Publisher per muovere il robot durante il visual servoing
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Inizializzazione servoing
        self.active_tag_id = 115     # ID del tag che stiamo cercando attualmente
        self.marker_detected = False
        self.marker_pose = None      # Posa del tag rispetto alla camera
        self.last_detection_time = 0
        self.state = STATE_SEARCH    # Stato iniziale
        self.last_error_yaw = 0.0
        
        # Parametri di movimento
        self.target_distance = 0.3         # Distanza target dal tag
        self.dist_tolerance = 0.02         # Tolleranza errore distanza
        self.yaw_tolerance_strict = 0.03   # Tolleranza allineamento (fase ALIGN)
        self.yaw_tolerance_loose = 0.12    # Tolleranza allineamento durante APPROACH
        
        # Guadagni del controllore P (Proporzionale)
        self.k_linear = 0.4
        self.k_angular = 0.8 
        self.max_linear_vel = 0.10  
        self.max_angular_vel = 0.25
        self.search_velocity = 0.20     # Velocità di rotazione durante la ricerca
        self.search_direction = 1.0     # 1.0 = SX, -1.0 = DX
        
        # --- CLIENT SERVIZI ---
        # Client per teletrasportare la scatola
        self.set_pose_client = self.create_client(SetEntityPose, '/world/warehouse/set_pose')

        # --- SYNC & GRIPPER ---
        # Interfaccia con Robot 1 (IIWA Pick)
        self.ready_pub_1 = self.create_publisher(Empty, TOPIC_SYNC_READY_1, 10)
        self.iiwa_sub_1 = self.create_subscription(Empty, TOPIC_SYNC_DONE_1, self.iiwa_1_done_callback, 10)
        self.iiwa_1_finished = False
        
        # Interfaccia con Robot 2 (IIWA Place/Shelf)
        self.ready_pub_2 = self.create_publisher(Empty, TOPIC_SYNC_READY_2, 10)
        self.iiwa_sub_2 = self.create_subscription(Empty, TOPIC_SYNC_DONE_2, self.iiwa_2_done_callback, 10)
        self.iiwa_2_finished = False

        # Sincronizzazione Handshake per IIWA 2
        self.iiwa_picked_sub = self.create_subscription(Empty, TOPIC_SYNC_IIWA_PICKED, self.iiwa_picked_callback, 10)
        self.iiwa_2_has_picked = False
        
        # Controllo Gripper (Attach/Detach simulato in Gazebo)
        self.attach_pub = self.create_publisher(Empty, TOPIC_FRA2MO_ATTACH, 10)
        self.detach_pub = self.create_publisher(Empty, TOPIC_FRA2MO_DETACH, 10)

        # Buffer TF per calcolare le trasformate (teletrasporto)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # =========================================================================
    # CALLBACKS
    # =========================================================================
    def aruco_1_callback(self, msg):
        # Elabora solo se stiamo cercando il Tag 115
        if self.active_tag_id == 115: self.process_aruco(msg)

    def aruco_2_callback(self, msg):
        # Elabora solo se stiamo cercando il Tag 201
        if self.active_tag_id == 201: self.process_aruco(msg)

    def iiwa_picked_callback(self, msg):
        # Handshaking: IIWA 2 dice "L'ho preso!". Solo ora possiamo rilasciare.
        self.get_logger().info(">> SYNC: IIWA 2 ha afferrato il pacco. Rilascio il mio gripper.")
        self.iiwa_2_has_picked = True

    def process_aruco(self, msg):
        """Salva la posa del marker e aggiorna il timestamp dell'ultimo avvistamento."""
        self.marker_detected = True
        self.marker_pose = msg.pose
        self.last_detection_time = self.get_clock().now().nanoseconds

    def get_yaw_error(self):
        """Calcola l'angolo (yaw) relativo al marker."""
        # atan2(y, x) ci dà l'angolo del vettore che punta al marker nel frame della camera
        return math.atan2(self.marker_pose.position.y, self.marker_pose.position.x)

    def iiwa_1_done_callback(self, msg):
        self.get_logger().info(">> SYNC: IIWA 1 DONE.")
        self.iiwa_1_finished = True

    def iiwa_2_done_callback(self, msg):
        self.get_logger().info(">> SYNC: IIWA 2 DONE.")
        self.iiwa_2_finished = True

    # =========================================================================
    # AZIONI ROBOT
    # =========================================================================
    def activate_gripper(self):
        """Dice a Gazebo di attaccare l'oggetto più vicino al robot."""
        self.get_logger().info(">> FRA2MO: ATTACH.")
        self.attach_pub.publish(Empty())
        time.sleep(1.0)

    def deactivate_gripper(self):
        """Dice a Gazebo di rilasciare qualsiasi oggetto attaccato."""
        self.get_logger().info(">> FRA2MO: DETACH.")
        self.detach_pub.publish(Empty())
        time.sleep(0.5)

    def reset_servoing(self, tag_id, distance_target=0.3):
        """Resetta la macchina a stati per iniziare una nuova ricerca."""
        self.active_tag_id = tag_id
        self.target_distance = distance_target
        self.state = STATE_SEARCH
        self.marker_detected = False
        self.marker_pose = None
        self.search_direction = 1.0
        self.get_logger().info(f"--- RESET SERVOING: Tag {tag_id} ---")

    def servoing_step(self):
        """
        Esegue un passo del loop di controllo visivo.
        Ritorna True se il target è raggiunto (STATE_FINISHED).
        """
        cmd = Twist()
        now = self.get_clock().now().nanoseconds
        # Calcola quanto tempo è passato dall'ultimo avvistamento (in secondi)
        time_since_detection = (now - self.last_detection_time) / 1e9
        is_detected_now = (time_since_detection < 0.5)

        if self.state == STATE_FINISHED:
            self.vel_pub.publish(Twist()) # Ferma il robot
            return True

        # --- STATO 0: RICERCA ---
        if self.state == STATE_SEARCH:
            if self.marker_detected and is_detected_now:
                self.vel_pub.publish(Twist()) # Stop momentaneo
                self.state = STATE_ALIGN      # Passa all'allineamento
                return False
            # Se non vedo nulla, ruoto su me stesso
            cmd.angular.z = self.search_velocity * self.search_direction
        
        # --- STATO 1: ALLINEAMENTO ANGOLARE ---
        elif self.state == STATE_ALIGN:
            if is_detected_now:
                error_yaw = self.get_yaw_error()
                self.last_error_yaw = error_yaw
                # Controllo P semplice sull'angolo
                cmd.angular.z = max(min(self.k_angular * error_yaw, self.max_angular_vel), -self.max_angular_vel)
                # Se l'errore è piccolo, passa all'approccio
                if abs(error_yaw) < self.yaw_tolerance_strict:
                    self.state = STATE_APPROACH
            else:
                # Se perdo il tag mentre mi allineo, provo a tornare indietro un po'
                recovery_speed = -1.0 * (self.search_velocity * self.search_direction) * 1.5
                cmd.angular.z = recovery_speed
                if time_since_detection > 3.0:
                    self.state = STATE_SEARCH

        # --- STATO 2: AVVICINAMENTO ---
        elif self.state == STATE_APPROACH:
            if not is_detected_now:
                self.vel_pub.publish(Twist())
                self.state = STATE_ALIGN # Perso il tag, mi riallineo
                return False

            current_dist = self.marker_pose.position.x
            error_dist = current_dist - self.target_distance
            error_yaw = self.get_yaw_error()

            # Se mentre avanzo mi storto troppo, torna ad allineare
            if abs(error_yaw) > self.yaw_tolerance_loose:
                self.state = STATE_ALIGN
                return False
            
            # Controllo P su distanza (Lineare X) e angolo (Angolare Z)
            cmd.linear.x = max(min(self.k_linear * error_dist, self.max_linear_vel), -self.max_linear_vel)
            cmd.angular.z = max(min(self.k_angular * error_yaw, self.max_angular_vel), -self.max_angular_vel)

            # Se siamo abbastanza vicini, FINITO
            if abs(error_dist) < self.dist_tolerance:
                self.get_logger().info(">> TARGET RAGGIUNTO.")
                self.state = STATE_FINISHED

        self.vel_pub.publish(cmd)
        return False

    def teletransport_box_to_fra2mo(self):
        """
        Calcola la posizione del vassoio del robot nella mappa globale e chiama il servizio Gazebo per spostare la scatola lì.
        """
        # 1. Calcolo della posizione target
        try:
            # Definiamo dove vogliamo la scatola RISPETTO AL ROBOT (base_footprint)
            box_pose_local = PoseStamped()
            box_pose_local.header.frame_id = 'base_footprint'
            box_pose_local.header.stamp = self.get_clock().now().to_msg()
            
            box_pose_local.pose.position.x = -0.08  # Leggermente indietro rispetto al centro
            box_pose_local.pose.position.y = 0.0    
            box_pose_local.pose.position.z = 0.3    # Altezza del vassoio
            box_pose_local.pose.orientation.w = 1.0 

            # Trasformiamo questa posizione in coordinate 'map' (Mondo)
            # lookup_transform ci da la posizione attuale del robot nella mappa
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            box_pose_map = do_transform_pose(box_pose_local.pose, transform)

            # 2. Creazione della richiesta di Servizio ROS
            req = SetEntityPose.Request()
            req.entity.name = 'box_blue'
            req.pose = box_pose_map      # Posa calcolata in coordinate globali

            # 3. Chiamata al service
            if not self.set_pose_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("Servizio set_pose non disponibile!")
                return

            # Inviamo la richiesta
            self.future = self.set_pose_client.call_async(req)
            self.get_logger().info(f">> MAGIC: Richiesta teletrasporto inviata a {box_pose_map.position.x:.2f}, {box_pose_map.position.y:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error teleport setup: {e}")

def main():
    rclpy.init()
    
    # Inizializziamo il Navigatore standard di Nav2
    navigator = BasicNavigator()
    # Inizializziamo il nostro nodo custom
    servoing_node = VisualServoingNode()
    
    # === SAFETY CHECK INIZIALE ===
    # Forziamo il detach
    print(">> SAFETY: Disattivazione preventiva di tutti i gripper...")
    for _ in range(10):
        servoing_node.deactivate_gripper()
        time.sleep(0.1)
    
    # =========================================================================
    # FASE 1: PRELIEVO (PICK)
    # =========================================================================
    print("\n>>> FASE 1: Navigazione verso IIWA 1 <<<")
    
    # 1. Navigazione verso l'area di prelievo
    goal_1 = PoseStamped()
    goal_1.header.frame_id = 'map'
    goal_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_1.pose.position.x = 3.5 
    goal_1.pose.position.y = 2.4 
    goal_1.pose.orientation.z = 0.707
    goal_1.pose.orientation.w = 0.707
    navigator.goToPose(goal_1)
    
    # Attesa arrivo Nav2
    while not navigator.isTaskComplete(): pass

    # 2. Avvicinamento fine (Visual Servoing) sul Tag 115
    print(">>> Servoing Tag 115 <<<")
    servoing_node.reset_servoing(115, 0.3)
    while rclpy.ok():
        rclpy.spin_once(servoing_node, timeout_sec=0.05)
        if servoing_node.servoing_step(): break # Esce quando STATE_FINISHED

    # 3. Sincronizzazione con IIWA 1
    print(">> Sync IIWA 1...")
    for _ in range(5):
        servoing_node.ready_pub_1.publish(Empty()) # "Sono qui!"
        time.sleep(0.2)
        
    # Attesa che IIWA 1 finisca la sua animazione/task
    while not servoing_node.iiwa_1_finished:
        rclpy.spin_once(servoing_node, timeout_sec=0.1)
    
    # 4. Caricamento Scatola (Teletrasporto + Attivazione Gripper)
    time.sleep(0.5)
    servoing_node.teletransport_box_to_fra2mo() # Sposta la scatola sul vassoio
    time.sleep(0.1)
    servoing_node.activate_gripper()            # "Incolla" la scatola

    # Pulizia mappe (per evitare che il robot veda se stesso o la scatola come ostacolo)
    navigator.clearAllCostmaps()
    time.sleep(1.0)

    # =========================================================================
    # FASE 2: CONSEGNA (PLACE)
    # =========================================================================
    print("\n>>> FASE 2: Navigazione verso IIWA 2 <<<")
    
    # 1. Navigazione verso l'area dello scaffale
    goal_2 = PoseStamped()
    goal_2.header.frame_id = 'map'
    goal_2.header.stamp = navigator.get_clock().now().to_msg()
    goal_2.pose.position.x = -3.85
    goal_2.pose.position.y = -3.6
    goal_2.pose.orientation.z = 1.0
    goal_2.pose.orientation.w = 0.0 
    navigator.goToPose(goal_2)
    
    # Durante il viaggio, continuiamo a spinnare il nodo per aggiornare TF e topic
    while not navigator.isTaskComplete(): 
        rclpy.spin_once(servoing_node, timeout_sec=0.1)

    # 2. Avvicinamento fine sul Tag 201
    print(">>> Servoing Tag 201 <<<")
    servoing_node.reset_servoing(201, 0.45)
    
    while rclpy.ok():
        rclpy.spin_once(servoing_node, timeout_sec=0.05)
        if servoing_node.servoing_step(): break

    # 3. Handshake con IIWA 2
    print(">> Sync IIWA 2: In posizione. Attendo che IIWA afferri...")
    
    # Invio segnale "Sono arrivato"
    for _ in range(5):
        servoing_node.ready_pub_2.publish(Empty())
        time.sleep(0.2)

    # Loop di attesa: Non mollo la presa finché IIWA 2 non conferma di aver preso
    wait_start = time.time()
    while not servoing_node.iiwa_2_has_picked:
        rclpy.spin_once(servoing_node, timeout_sec=0.1)
        # Timeout di sicurezza: Ripubblichiamo "Sono qui" ogni 2 secondi
        if (time.time() - wait_start) > 2.0:
            servoing_node.ready_pub_2.publish(Empty())
            wait_start = time.time()

    # Conferma ricevuta -> Detach
    print(">> Conferma ricevuta. Rilascio il pacco.")
    servoing_node.deactivate_gripper()

    # Attesa fine movimento IIWA 2
    while not servoing_node.iiwa_2_finished:
        rclpy.spin_once(servoing_node, timeout_sec=0.1)

    # =========================================================================
    # FASE 3: RITORNO (HOME)
    # =========================================================================
    print("\n>>> Ritorno Home <<<")
    home = PoseStamped()
    home.header.frame_id = 'map'
    home.header.stamp = navigator.get_clock().now().to_msg()
    home.pose.position.x = 0.0; home.pose.position.y = 0.0; home.pose.orientation.w = 1.0
    navigator.goToPose(home)
    while not navigator.isTaskComplete(): rclpy.spin_once(servoing_node, timeout_sec=0.1)

    print("MISSIONE COMPLETA.")
    servoing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
