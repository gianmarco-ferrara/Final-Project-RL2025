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

# =============================================================================
# CONFIGURAZIONE
# =============================================================================
# Il nome dell'azione standard esposta dal controller è <controller_name>/follow_joint_trajectory
ACTION_NAME = '/iiwa/iiwa_1_arm_controller/follow_joint_trajectory'

TOPIC_ATTACH = '/iiwa/picker/attach'
TOPIC_DETACH = '/iiwa/picker/detach'

TOPIC_SYNC_READY    = '/coordination/fra2mo_ready'
TOPIC_SYNC_DONE     = '/coordination/iiwa_done'

JOINT_NAMES = [
    'iiwa_joint_a1', 'iiwa_joint_a2', 'iiwa_joint_a3', 'iiwa_joint_a4', 
    'iiwa_joint_a5', 'iiwa_joint_a6', 'iiwa_joint_a7'
]

POSES = {
    "HOME":  [0.625, 0.713, 0.240, -0.487, -0.112, 0.487, 0.00],
    "HIGH": [-1.139, 0.713, 0.208, -0.509, -0.176, 0.487, 0.00],
    "PICK":  [0.593, 1.664, 0.008, -0.079, 0.00, 0.849, 0.00], 
    "PLACE": [-1.395, 1.574, -0.112, -1.211, 0.080, 0.328, -0.016] 
}

class IIWACoordinatedAction(Node):
    def __init__(self):
        super().__init__('iiwa1_node_action')
        
        # Action Client per il movimento
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            ACTION_NAME
        )
        
        # Publisher Gripper
        self.attach_pub = self.create_publisher(Empty, TOPIC_ATTACH, 10)
        self.detach_pub = self.create_publisher(Empty, TOPIC_DETACH, 10)
        
        # Sync
        self.done_pub = self.create_publisher(Empty, TOPIC_SYNC_DONE, 10)
        self.ready_sub = self.create_subscription(Empty, TOPIC_SYNC_READY, self.ready_callback, 10)
        self.fra2mo_is_ready = False
        
        self.get_logger().info('Nodo IIWA (Action Client) Inizializzato.')

    def ready_callback(self, msg):
        if not self.fra2mo_is_ready:
            self.get_logger().info(">> SYNC: Fra2mo in posizione.")
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
        Invia la traiettoria tramite Action e attende il completamento.
        """
        # 1. Attesa connessione server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action Server {ACTION_NAME} non disponibile!")
            return False

        target_pos = POSES[pose_name]
        
        # 2. Creazione del Goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = target_pos
        point.time_from_start = Duration(seconds=int(duration_sec), nanoseconds=0).to_msg()
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'>> Sending Action Goal: {pose_name} (T={duration_sec}s)...')

        # 3. Invio Goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # Blocchiamo finché il server non accetta il goal
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rifiutato dal server!')
            return False

        self.get_logger().info('Goal accettato, movimento in corso...')

        # 4. Attesa del Risultato
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        status = get_result_future.result().status

        # Status 4 = SUCCEEDED (definito in action_msgs/msg/GoalStatus)
        if status == 4: 
            self.get_logger().info(f'Movimento {pose_name} completato con successo (Error code: {result.error_code})')
            return True
        else:
            self.get_logger().warn(f'Movimento fallito con status: {status}')
            return False

    def attach(self):
        self.get_logger().info('>> GRIPPER: Attach.')
        self.attach_pub.publish(Empty())
        time.sleep(1.0)

    def detach(self):
        self.get_logger().info('>> GRIPPER: Detach.')
        self.detach_pub.publish(Empty())
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = IIWACoordinatedAction()

    try:
        print(">> SAFETY: Disattivazione preventiva di tutti i gripper...")
        for _ in range(10):
            node.detach()
            time.sleep(0.1)
        
        # 1. Reset
        print("\n--- POSIZIONAMENTO INIZIALE ---")
        node.detach()
        node.move_to("HOME", duration_sec=4.0)

        # 2. Sync
        print(">> IN ATTESA CHE FRA2MO SIA PRONTO... <<")
        while not node.fra2mo_is_ready:
            # Usiamo spin_once breve per controllare la callback del sync
            rclpy.spin_once(node, timeout_sec=0.1)

        print("\n--- INIZIO SEQUENZA ---")
        
        # 3. Pick
        node.move_to("PICK", duration_sec=4.0)
        
        time.sleep(1.0)
        node.attach()
        time.sleep(1.0)
        
        # 4. Move Up & Wait
        node.move_to("HOME", duration_sec=4.0)
        node.move_to("HIGH", duration_sec=4.0)
        
        # 5. Place
        node.move_to("PLACE", duration_sec=5.0)
        
        print("--- RILASCIO ---")
        time.sleep(0.5)
        node.detach()
        time.sleep(2.0)
        
        print(">> INVIO SEGNALE: Done. <<")
        node.done_pub.publish(Empty())

        time.sleep(1.0)
        node.move_to("HOME", duration_sec=4.0)
        
        print("--- COMPLETATO ---")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()