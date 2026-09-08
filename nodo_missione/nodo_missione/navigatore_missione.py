import math
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

WAYPOINTS = [
    (-3.0, 1.0),
    (1.0, -2.8),
    (3.7, -1.8),
    (3.5, 1.4),
]

DURATA_SCANSIONE = 20.0
VELOCITA_ANGOLARE_SCANSIONE = 0.5
CARTELLA_SCANSIONI = os.path.expanduser('~/rover_ws/scansioni')


class NavigatoreMissione(Node):
    def __init__(self):
        super().__init__('navigatore_missione')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)

        self.bridge_immagini = CvBridge()
        self.ultimo_frame = None
        self.create_subscription(Image, '/camera/image', self.callback_camera, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        os.makedirs(CARTELLA_SCANSIONI, exist_ok=True)

        self.indice_waypoint = 0
        self.timer_scansione = None
        self.yaw_iniziale_scansione = None
        self.prossima_soglia_foto = 0
        self.soglie_foto_gradi = [0, 90, 180, 270]

        self.get_logger().info('Nodo di missione avviato, attendo il server di navigazione...')
        self.action_client.wait_for_server()
        self.get_logger().info('Server di navigazione disponibile, avvio la missione.')
        self.invia_prossimo_waypoint()

    def callback_camera(self, msg):
        self.ultimo_frame = msg

    def leggi_yaw_attuale(self):
        try:
            trasformata = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        q = trasformata.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw

    def invia_prossimo_waypoint(self):
        if self.indice_waypoint >= len(WAYPOINTS):
            self.get_logger().info('Missione completata: tutti i waypoint raggiunti.')
            return

        x, y = WAYPOINTS[self.indice_waypoint]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Invio waypoint {self.indice_waypoint + 1}/{len(WAYPOINTS)}: ({x}, {y})'
        )

        future_invio = self.action_client.send_goal_async(goal_msg)
        future_invio.add_done_callback(self.callback_risposta_goal)

    def callback_risposta_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rifiutato dal server di navigazione.')
            return

        future_risultato = goal_handle.get_result_async()
        future_risultato.add_done_callback(self.callback_risultato_navigazione)

    def callback_risultato_navigazione(self, future):
        stato = future.result().status
        if stato == 4:
            self.get_logger().info(
                f'Waypoint {self.indice_waypoint + 1} raggiunto con successo. Avvio scansione.'
            )
            self.avvia_scansione()
        else:
            self.get_logger().warn(
                f'Waypoint {self.indice_waypoint + 1} non raggiunto (stato {stato}). Passo al successivo.'
            )
            self.indice_waypoint += 1
            self.invia_prossimo_waypoint()

    def avvia_scansione(self):
        self.yaw_precedente = self.leggi_yaw_attuale()
        self.rotazione_accumulata_gradi = 0.0
        self.prossima_soglia_foto = 0
        self.timer_scansione = self.create_timer(0.05, self.callback_scansione)
        self.tempo_inizio_scansione = self.get_clock().now()

    def callback_scansione(self):
        tempo_trascorso = (
            self.get_clock().now() - self.tempo_inizio_scansione
        ).nanoseconds / 1e9

        if tempo_trascorso >= DURATA_SCANSIONE:
            self.cmd_vel_pub.publish(Twist())
            self.timer_scansione.cancel()
            self.get_logger().info(
                f'Scansione al waypoint {self.indice_waypoint + 1} completata.'
            )
            self.indice_waypoint += 1
            self.invia_prossimo_waypoint()
            return

        comando = Twist()
        comando.angular.z = VELOCITA_ANGOLARE_SCANSIONE
        self.cmd_vel_pub.publish(comando)

        if self.prossima_soglia_foto < len(self.soglie_foto_gradi):
            yaw_attuale = self.leggi_yaw_attuale()
            if yaw_attuale is not None and self.yaw_precedente is not None:
                diff = yaw_attuale - self.yaw_precedente
                diff_gradi = math.degrees(math.atan2(math.sin(diff), math.cos(diff)))
                self.rotazione_accumulata_gradi += diff_gradi
                self.yaw_precedente = yaw_attuale

                soglia_attuale = self.soglie_foto_gradi[self.prossima_soglia_foto]
                if soglia_attuale == 0:
                    self.scatta_foto(soglia_attuale)
                    self.prossima_soglia_foto += 1
                elif self.rotazione_accumulata_gradi >= soglia_attuale:
                    self.scatta_foto(soglia_attuale)
                    self.prossima_soglia_foto += 1

    def scatta_foto(self, angolo_gradi):
        if self.ultimo_frame is None:
            self.get_logger().warn(f'Nessun frame camera disponibile per angolo {angolo_gradi}.')
            return
        try:
            immagine_cv = self.bridge_immagini.imgmsg_to_cv2(self.ultimo_frame, desired_encoding='bgr8')
            nome_file = os.path.join(
                CARTELLA_SCANSIONI,
                f'waypoint_{self.indice_waypoint + 1}_angolo_{angolo_gradi}.png'
            )
            cv2.imwrite(nome_file, immagine_cv)
            self.get_logger().info(f'Foto salvata: {nome_file}')
        except Exception as e:
            self.get_logger().error(f'Errore nel salvataggio della foto: {e}')


def main():
    rclpy.init()
    nodo = NavigatoreMissione()
    rclpy.spin(nodo)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
