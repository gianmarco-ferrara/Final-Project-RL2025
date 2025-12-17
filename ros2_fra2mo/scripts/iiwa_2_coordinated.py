#! /usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import Empty
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import rclpy.exceptions

# =============================================================================
# CONFIGURAZIONE IIWA
# =============================================================================
# Nome dell'azione per il controller di IIWA 2
ACTION_NAME = '/iiwa_2/iiwa_2_arm_controller/follow_joint_trajectory'

TOPIC_ATTACH = '/iiwa_2/picker/attach'
TOPIC_DETACH = '/iiwa_2/picker/detach'

# Topics sincronizzazione
TOPIC_SYNC_FRA2MO_READY = '/coordination/fra2mo_at_shelf' 
TOPIC_SYNC_IIWA_DONE    = '/coordination/iiwa_2_done'
TOPIC_SYNC_IIWA_PICKED  = '/coordination/iiwa_2_has_picked'

JOINT_NAMES = [
    'iiwa_2_joint_a1', 'iiwa_2_joint_a2', 'iiwa_2_joint_a3', 'iiwa_2_joint_a4', 
    'iiwa_2_joint_a5', 'iiwa_2_joint_a6', 'iiwa_2_joint_a7'
]

POSES = {
    "HOME":  [0.0, 0.35, 0.0, -1.57, 0.0, 1.57, 0.0],
    "PICK_FROM_ROBOT": [-0.112, 1.936, -0.337, -0.192, -0.080, 0.849, 0.0],
    "PLACE_SHELF": [-1.139, 1.034, -0.337, -0.170, 0.0, 0.306, 0.0],
    "HIGH": [-0.850, 0.555, -0.337, -0.192, -0.080, 0.849, 0.0],
}

class IIWA2CoordinatedAction(Node):
    def __init__(self):
        super().__init__('iiwa2_node_action')
        
        # Action Client
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            ACTION_NAME
        )
        
        # Publisher Gripper
        self.attach_pub = self.create_publisher(Empty, TOPIC_ATTACH, 10)
        self.detach_pub = self.create_publisher(Empty, TOPIC_DETACH, 10)
        
        # Sincronizzazione
        self.done_pub = self.create_publisher(Empty, TOPIC_SYNC_IIWA_DONE, 10)
        self.picked_pub = self.create_publisher(Empty, TOPIC_SYNC_IIWA_PICKED, 10)
        
        self.ready_sub = self.create_subscription(Empty, TOPIC_SYNC_FRA2MO_READY, self.ready_callback, 10)
        self.fra2mo_is_ready = False
        
        self.get_logger().info('Nodo IIWA 2 (Action Client) pronto.')

    def ready_callback(self, msg):
        if not self.fra2mo_is_ready:
            self.get_logger().info(">> IIWA 2: Fra2mo arrivato allo scaffale!")
            self.fra2mo_is_ready = True

    def feedback_callback(self, feedback_msg):
        """
        Questa callback viene chiamata periodicamente dal server durante il movimento.
        feedback_msg.feedback contiene:
         - header
         - joint_names
         - desired
         - actual
         - error
        """
        feedback = feedback_msg.feedback
        
        # Verifichiamo che ci siano dati sull'errore
        if feedback.error and feedback.error.positions:
            errors = feedback.error.positions
            n_joints = len(errors)
            
            if n_joints > 0:
                # 1. Somma dei quadrati degli errori
                sum_sq_errors = sum([e**2 for e in errors])
                
                # 2. Calcolo della Norma Euclidea (Radice quadrata della somma)
                euclidean_norm = math.sqrt(sum_sq_errors)
                
                # 3. Divisione per il numero di giunti
                result = euclidean_norm / n_joints
                
                self.get_logger().info(
                    f'>> Feedback: (Norma L2 / N) = {result:.6f} | Norma: {euclidean_norm:.4f}'
                )

    def move_to(self, pose_name, duration_sec=6.0):
        """
        Invia la traiettoria all'Action Server e attende il completamento.
        """
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action Server {ACTION_NAME} non disponibile!")
            return False

        target_pos = POSES[pose_name]
        
        # Costruzione del Goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = target_pos
        point.time_from_start = Duration(seconds=int(duration_sec), nanoseconds=0).to_msg()
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'>> Sending Action Goal: {pose_name} (T={duration_sec}s)...')

        # Invio asincrono
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # Attesa accettazione
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rifiutato dal server!')
            return False

        # Attesa risultato finale
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        status = get_result_future.result().status

        # Status 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info(f'Movimento {pose_name} completato. (ErrCode: {result.error_code})')
            return True
        else:
            self.get_logger().warn(f'Movimento fallito con status: {status}')
            return False

    def attach(self):
        self.get_logger().info('>> IIWA 2: ATTACH.')
        self.attach_pub.publish(Empty())
        time.sleep(0.5)

    def detach(self):
        self.get_logger().info('>> IIWA 2: DETACH.')
        self.detach_pub.publish(Empty())
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = IIWA2CoordinatedAction()

    try:
        print(">> SAFETY: Disattivazione preventiva di tutti i gripper...")
        for _ in range(10):
            node.detach()
            time.sleep(0.1)

        # 1. Posizione Home
        node.move_to("HOME", duration_sec=4.0)

        print(">> IIWA 2: In attesa di Fra2mo...")
        while not node.fra2mo_is_ready:
            rclpy.spin_once(node, timeout_sec=0.1)

        print("\n--- INIZIO PICK ---")
        node.move_to("PICK_FROM_ROBOT", duration_sec=5.0)
        
        # SEQUENZA CRITICA DI PRESA
        print(">> IIWA 2: Attivo presa (Attach)...")
        node.attach() 
        time.sleep(1.0)
        
        print(">> IIWA 2: Segnalo a Fra2mo che ho preso il pacco.")
        # Solo ORA diciamo a Fra2mo che può lasciare
        for _ in range(3):
            node.picked_pub.publish(Empty())
            time.sleep(0.1)
        
        # Aspettiamo che Fra2mo esegua il detach e si stabilizzi
        time.sleep(2.0)
        
        print("--- INIZIO PLACE ---")
        node.move_to("HIGH", duration_sec=4.0) 
        node.move_to("PLACE_SHELF", duration_sec=6.0)
        
        time.sleep(0.5)
        node.detach()
        time.sleep(1.0)
        
        node.move_to("HIGH", duration_sec=3.0)
        node.move_to("HOME", duration_sec=3.0)
        
        node.done_pub.publish(Empty())
        print(">> IIWA 2: Missione Completata.")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()