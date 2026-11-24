import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float64MultiArray

from tigo_base.serial_protocol import (
    MecanumOdomState,
    parse_encoder_line,
    parse_pid_line,
    update_mecanum_odom,
)


class ArduinoBridge(Node):
    """
    Ponte serial bidirecional entre ROS 2 e o Arduino Mega do robô TIGO.

    - Assina /cmd_vel e envia 'V,vx,vy,wz\\n'
    - Assina /tigo/pid_gains e envia 'G,kp,ki,kd\\n'
    - Lê encoders ('E,...') e publica /odom (nav_msgs/Odometry)
    - Lê telemetria PID ('P,...') e publica /tigo/pid_debug (Float64MultiArray)
    """

    def __init__(self):
        super().__init__('arduino_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.05)
        self.declare_parameter('wheel_radius', 0.041)
        self.declare_parameter('lx', 0.082)
        self.declare_parameter('ly', 0.106)
        self.declare_parameter('ticks_per_rev', 1319.1)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.lx = self.get_parameter('lx').get_parameter_value().double_value
        self.ly = self.get_parameter('ly').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').get_parameter_value().double_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        self._serial_lock = threading.Lock()
        self.serial_conn = None
        self._rx_buffer = ''
        self._odom_state = MecanumOdomState()
        self._last_odom_time = None

        self.connect_serial()

        self.reconnect_timer = self.create_timer(2.0, self.check_connection)
        self.serial_read_timer = self.create_timer(0.01, self.poll_serial)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pid_pub = self.create_publisher(Float64MultiArray, '/tigo/pid_debug', 10)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )
        self.gains_subscription = self.create_subscription(
            Float64MultiArray,
            '/tigo/pid_gains',
            self.pid_gains_callback,
            10,
        )

        self.get_logger().info(
            f'Nó arduino_bridge iniciado. Porta: {self.port} | Baud: {self.baudrate}'
        )

    def connect_serial(self):
        try:
            conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
            time.sleep(2.0)  # Aguarda o bootloader do Arduino estabilizar após conexão
            with self._serial_lock:
                self.serial_conn = conn
                self._rx_buffer = ''
                self._odom_state.prev_ticks = None
                self._last_odom_time = None
            self.get_logger().info(
                f'Conexão serial com o Arduino estabelecida na porta {self.port}!'
            )
        except serial.SerialException as e:
            self.get_logger().warn(
                f'Arduino não detectado em {self.port} ({e}). '
                'O nó tentará reconectar automaticamente quando conectado.'
            )
            with self._serial_lock:
                self.serial_conn = None

    def check_connection(self):
        with self._serial_lock:
            is_open = self.serial_conn is not None and self.serial_conn.is_open
        if not is_open:
            self.connect_serial()

    def _write_serial_line(self, line: str) -> None:
        with self._serial_lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(line.encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Falha ao enviar dados pela serial: {e}')
                    self.serial_conn = None

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self._write_serial_line(f'V,{vx:.3f},{vy:.3f},{wz:.3f}\n')

    def pid_gains_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            return
        kp = float(msg.data[0])
        ki = float(msg.data[1])
        kd = float(msg.data[2])
        if kp < 0.0 or ki < 0.0 or kd < 0.0:
            return
        self._write_serial_line(f'G,{kp:.4f},{ki:.4f},{kd:.4f}\n')
        self.get_logger().info(
            f'Ganhos PID enviados ao Arduino: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.4f}'
        )

    def poll_serial(self):
        chunk = b''
        with self._serial_lock:
            if self.serial_conn is None or not self.serial_conn.is_open:
                return
            try:
                waiting = self.serial_conn.in_waiting
                if waiting > 0:
                    chunk = self.serial_conn.read(waiting)
            except serial.SerialException as e:
                self.get_logger().error(f'Falha ao ler serial: {e}')
                self.serial_conn = None
                return

        if not chunk:
            return

        self._rx_buffer += chunk.decode('utf-8', errors='replace')
        while '\n' in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split('\n', 1)
            line = line.strip()
            if line:
                self._process_serial_line(line)

    def _process_serial_line(self, line: str) -> None:
        if line.startswith('E,'):
            ticks = parse_encoder_line(line)
            if ticks is None:
                return
            now = self.get_clock().now()
            dt = 0.0
            if self._last_odom_time is not None:
                dt = (now - self._last_odom_time).nanoseconds / 1e9
            self._last_odom_time = now

            vel = update_mecanum_odom(
                self._odom_state,
                ticks,
                dt,
                self.wheel_radius,
                self.lx,
                self.ly,
                self.ticks_per_rev,
            )
            if vel is not None:
                self._publish_odom(now, *vel)
        elif line.startswith('P,'):
            pid_vec = parse_pid_line(line)
            if pid_vec is None:
                return
            msg = Float64MultiArray()
            msg.data = pid_vec
            self.pid_pub.publish(msg)

    def _publish_odom(self, stamp, vx: float, vy: float, wz: float) -> None:
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id

        msg.pose.pose.position.x = self._odom_state.x
        msg.pose.pose.position.y = self._odom_state.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self._odom_state.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._odom_state.yaw / 2.0)

        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        self.odom_pub.publish(msg)

    def destroy_node(self):
        with self._serial_lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(b'V,0.000,0.000,0.000\n')
                    self.serial_conn.close()
                except Exception:
                    pass
                self.serial_conn = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
