import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class ArduinoBridge(Node):
    """
    Nó que assina o tópico /cmd_vel e envia os comandos de velocidade
    linear (x, y) e angular (z) via comunicação serial para o Arduino Mega.
    Formato da mensagem enviada: 'V,vx,vy,wz\\n'
    """

    def __init__(self):
        super().__init__('arduino_bridge')

        # Parâmetros configuráveis (podem ser alterados via launch ou linha de comando)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        self.serial_conn = None
        self.connect_serial()

        # Timer para tentar reconectar se a serial cair ou o cabo for plugado depois
        self.reconnect_timer = self.create_timer(2.0, self.check_connection)

        # Assinante de /cmd_vel (padrão ROS 2 para robôs móveis)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(
            f'Nó arduino_bridge iniciado. Porta: {self.port} | Baud: {self.baudrate}'
        )

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2.0)  # Aguarda o bootloader do Arduino estabilizar após conexão
            self.get_logger().info(f'Conexão serial com o Arduino estabelecida na porta {self.port}!')
        except serial.SerialException as e:
            self.get_logger().warn(
                f'Arduino não detectado em {self.port} ({e}). '
                'O nó tentará reconectar automaticamente quando conectado.'
            )
            self.serial_conn = None

    def check_connection(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.connect_serial()

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Formato esperado pelo sketch do Arduino: "V,vx,vy,wz\n"
        linha_comando = f"V,{vx:.3f},{vy:.3f},{wz:.3f}\n"

        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(linha_comando.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f'Falha ao enviar dados pela serial: {e}')
                self.serial_conn = None

    def destroy_node(self):
        # Envia comando de parada com segurança antes de fechar
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(b"V,0.000,0.000,0.000\n")
                self.serial_conn.close()
            except Exception:
                pass
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
