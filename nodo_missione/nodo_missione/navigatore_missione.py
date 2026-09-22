import math
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import csv
from datetime import datetime

CARTELLA_LOG = os.path.expanduser('~/rover_ws/log_missione')

WAYPOINTS = [
    (-3.0, 1.0),
    (1.0, -2.8),
    (3.7, -1.8),
    (3.5, 1.4),
]

DURATA_SCANSIONE = 20.0
VELOCITA_ANGOLARE_SCANSIONE = 0.5
RAGGIO_MASSIMO_NUVOLA_PUNTI = 6.0
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

        self.ultimo_scan = None
        self.scansione_attiva = False
        self.nuvola_punti_accumulata = []
        self.create_subscription(LaserScan, '/scan', self.callback_scan, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        os.makedirs(CARTELLA_SCANSIONI, exist_ok=True)
        os.makedirs(CARTELLA_LOG, exist_ok=True)
        timestamp_run = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.file_csv = os.path.join(CARTELLA_LOG, f'run_{timestamp_run}.csv')
        self.file_testo = os.path.join(CARTELLA_LOG, f'run_{timestamp_run}.txt')

        with open(self.file_csv, 'w', newline='') as f:
            scrittore = csv.writer(f)
            scrittore.writerow([
                'indice_waypoint', 'x', 'y', 'esito', 'tempo_navigazione_s',
                'numero_recoveries', 'foto_scattate', 'nuvola_salvata',
                'numero_punti_nuvola', 'errori'
            ])

        self.indice_waypoint = 0
        self.timer_scansione = None
        self.yaw_precedente = None
        self.rotazione_accumulata_gradi = 0.0
        self.prossima_soglia_foto = 0
        self.soglie_foto_gradi = [0, 90, 180, 270]

        self.log_evento('Avvio del nodo di missione')
        self.get_logger().info('Nodo di missione avviato, attendo il server di navigazione...')
        self.action_client.wait_for_server()
        self.get_logger().info('Server di navigazione disponibile, attendo stabilizzazione...')
        time.sleep(5)
        self.get_logger().info('Avvio la missione.')
        self.invia_prossimo_waypoint()

    def callback_camera(self, msg):
        self.ultimo_frame = msg

    def callback_scan(self, msg):
        self.ultimo_scan = msg
        if not self.scansione_attiva:
            return
        try:
            trasformata = self.tf_buffer.lookup_transform(
                'odom', msg.header.frame_id, rclpy.time.Time()
            )
        except Exception:
            return

        angolo = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= RAGGIO_MASSIMO_NUVOLA_PUNTI:
                x_locale = r * math.cos(angolo)
                y_locale = r * math.sin(angolo)
                xg, yg, zg = self.trasforma_punto_in_globale(x_locale, y_locale, trasformata)
                self.nuvola_punti_accumulata.append((xg, yg, zg))
            angolo += msg.angle_increment

    def trasforma_punto_in_globale(self, x_locale, y_locale, trasformata):
        tx = trasformata.transform.translation.x
        ty = trasformata.transform.translation.y
        tz = trasformata.transform.translation.z
        q = trasformata.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x_globale = tx + x_locale * math.cos(yaw) - y_locale * math.sin(yaw)
        y_globale = ty + x_locale * math.sin(yaw) + y_locale * math.cos(yaw)
        return x_globale, y_globale, tz

    def leggi_yaw_attuale(self):
        try:
            trasformata = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        q = trasformata.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw
    
    def log_evento(self, messaggio):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        riga = f'[{timestamp}] {messaggio}\n'
        with open(self.file_testo, 'a') as f:
            f.write(riga)

    def scrivi_riga_csv(self, x, y, foto_scattate, nuvola_salvata, numero_punti, errori):
        with open(self.file_csv, 'a', newline='') as f:
            scrittore = csv.writer(f)
            scrittore.writerow([
                self.indice_waypoint + 1, x, y, self.esito_corrente,
                f'{self.tempo_navigazione_corrente:.1f}',
                self.numero_recoveries_corrente,
                foto_scattate, nuvola_salvata, numero_punti, errori
            ])
        self.indice_waypoint += 1
        self.invia_prossimo_waypoint()

    def invia_prossimo_waypoint(self):
        if self.indice_waypoint >= len(WAYPOINTS):
            self.get_logger().info('Missione completata: tutti i waypoint raggiunti.')
            self.log_evento('Missione completata: tutti i waypoint raggiunti.')
            rclpy.shutdown()
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
        self.log_evento(f'Invio waypoint {self.indice_waypoint + 1}/{len(WAYPOINTS)}: ({x}, {y})')

        self.tempo_inizio_navigazione = self.get_clock().now()
        self.numero_recoveries_corrente = 0
        future_invio = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.callback_feedback_navigazione
        )
        future_invio.add_done_callback(self.callback_risposta_goal)

    def callback_feedback_navigazione(self, feedback_msg):
        self.numero_recoveries_corrente = feedback_msg.feedback.number_of_recoveries

    def callback_risposta_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rifiutato dal server di navigazione.')
            self.log_evento(f'ERRORE: goal rifiutato per il waypoint {self.indice_waypoint + 1}, passo al successivo.')
            self.esito_corrente = 'rifiutato'
            self.tempo_navigazione_corrente = 0.0
            self.numero_recoveries_corrente = 0
            self.scrivi_riga_csv(*WAYPOINTS[self.indice_waypoint], foto_scattate=0, nuvola_salvata=False, numero_punti=0, errori='goal rifiutato')
            return

        future_risultato = goal_handle.get_result_async()
        future_risultato.add_done_callback(self.callback_risultato_navigazione)

    def callback_risultato_navigazione(self, future):
        stato = future.result().status
        self.tempo_navigazione_corrente = (
            self.get_clock().now() - self.tempo_inizio_navigazione
        ).nanoseconds / 1e9

        if stato == 4:
            self.esito_corrente = 'successo'
            self.get_logger().info(
                f'Waypoint {self.indice_waypoint + 1} raggiunto con successo. Avvio scansione.'
            )
            self.log_evento(
                f'Waypoint {self.indice_waypoint + 1} raggiunto: successo, '
                f'tempo={self.tempo_navigazione_corrente:.1f}s, recovery={self.numero_recoveries_corrente}'
            )
            self.avvia_scansione()
        else:
            self.esito_corrente = f'fallito (stato {stato})'
            self.get_logger().warn(
                f'Waypoint {self.indice_waypoint + 1} non raggiunto (stato {stato}). Passo al successivo.'
            )
            self.log_evento(
                f'Waypoint {self.indice_waypoint + 1} non raggiunto: {self.esito_corrente}, '
                f'tempo={self.tempo_navigazione_corrente:.1f}s, recovery={self.numero_recoveries_corrente}'
            )
            self.scrivi_riga_csv(foto_scattate=0, nuvola_salvata=False, numero_punti=0, errori='')
            self.indice_waypoint += 1
            self.invia_prossimo_waypoint()

    def avvia_scansione(self):
        self.yaw_precedente = self.leggi_yaw_attuale()
        self.rotazione_accumulata_gradi = 0.0
        self.prossima_soglia_foto = 0
        self.foto_scattate_contatore = 0
        self.nuvola_punti_accumulata = []
        self.scansione_attiva = True
        self.timer_scansione = self.create_timer(0.05, self.callback_scansione)
        self.tempo_inizio_scansione = self.get_clock().now()

    def callback_scansione(self):
        tempo_trascorso = (
            self.get_clock().now() - self.tempo_inizio_scansione
        ).nanoseconds / 1e9

        if tempo_trascorso >= DURATA_SCANSIONE:
            self.cmd_vel_pub.publish(Twist())
            self.timer_scansione.cancel()
            self.scansione_attiva = False
            self.get_logger().info(
                f'Scansione al waypoint {self.indice_waypoint + 1} completata.'
            )
            self.log_evento(f'Scansione al waypoint {self.indice_waypoint + 1} completata.')
            self.salva_nuvola_punti()
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
            self.foto_scattate_contatore += 1
            self.log_evento(f'Foto salvata (angolo {angolo_gradi}°): {nome_file}')
        except Exception as e:
            self.get_logger().error(f'Errore nel salvataggio della foto: {e}')
            self.log_evento(f'ERRORE nel salvataggio della foto (angolo {angolo_gradi}°): {e}')

    def salva_nuvola_punti(self):
        punti = self.nuvola_punti_accumulata
        nome_file = os.path.join(
            CARTELLA_SCANSIONI,
            f'waypoint_{self.indice_waypoint + 1}_nuvola.pcd'
        )
        nuvola_salvata = False
        errori = ''
        try:
            with open(nome_file, 'w') as f:
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\n')
                f.write('FIELDS x y z\n')
                f.write('SIZE 4 4 4\n')
                f.write('TYPE F F F\n')
                f.write('COUNT 1 1 1\n')
                f.write(f'WIDTH {len(punti)}\n')
                f.write('HEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {len(punti)}\n')
                f.write('DATA ascii\n')
                for x, y, z in punti:
                    f.write(f'{x:.4f} {y:.4f} {z:.4f}\n')
            self.get_logger().info(f'Nuvola di punti salvata: {nome_file} ({len(punti)} punti)')
            self.log_evento(f'Nuvola di punti salvata: {nome_file} ({len(punti)} punti)')
            nuvola_salvata = True
        except Exception as e:
            self.get_logger().error(f'Errore nel salvataggio della nuvola di punti: {e}')
            self.log_evento(f'ERRORE nel salvataggio della nuvola di punti: {e}')
            errori = str(e)

        x, y = WAYPOINTS[self.indice_waypoint]
        self.scrivi_riga_csv(
            x, y,
            foto_scattate=self.foto_scattate_contatore,
            nuvola_salvata=nuvola_salvata,
            numero_punti=len(punti),
            errori=errori
        )


def main():
    rclpy.init()
    nodo = NavigatoreMissione()
    rclpy.spin(nodo)


if __name__ == '__main__':
    main()
