import importlib.util
import threading
from typing import Optional, Tuple

from core.kinematics import quaternion_to_yaw
from core.models import ConnectionConfig, ConnectionState
from providers.base import TelemetryCallbacks, TelemetryProvider, parse_pid_array


def is_rclpy_available() -> bool:
    """Verifica se o ambiente Python atual possui ROS 2 (rclpy) carregado."""
    return importlib.util.find_spec("rclpy") is not None


class Ros2NativeClient(TelemetryProvider):
    """
    Assinante nativo de tópicos ROS 2 via rclpy (dispensa rosbridge quando executado
    no mesmo computador/domínio ROS 2 após `source /opt/ros/humble/setup.bash`).
    """

    def __init__(self, config: ConnectionConfig, callbacks: TelemetryCallbacks) -> None:
        self._config = config
        self._callbacks = callbacks
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._cmd_lock = threading.Lock()
        self._pending_cmd: Optional[Tuple[float, float, float]] = None
        self._pending_gains: Optional[Tuple[float, float, float]] = None

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        with self._cmd_lock:
            self._pending_cmd = (vx, vy, wz)

    def send_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        with self._cmd_lock:
            self._pending_gains = (kp, ki, kd)

    def start(self) -> None:
        if not is_rclpy_available():
            self._callbacks.on_status(
                ConnectionState.ERROR,
                "rclpy não encontrado neste terminal. Carregue o ROS 2 ou use rosbridge/Mock.",
            )
            return
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._callbacks.on_status(
            ConnectionState.CONNECTING, "Iniciando nó assinante ROS 2 (rclpy)..."
        )
        self._thread = threading.Thread(target=self._spin_worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None
        self._callbacks.on_status(
            ConnectionState.DISCONNECTED, "Nó ROS 2 (rclpy) encerrado"
        )

    def _spin_worker(self) -> None:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray

        created_context = False
        node = None
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                created_context = True

            topics = self._config.topics
            callbacks = self._callbacks

            class _SubscriberNode(Node):
                def __init__(self) -> None:
                    super().__init__("tigo_telemetry_monitor")
                    self.cmd_pub = self.create_publisher(Twist, topics.cmd_vel, 10)
                    self.gains_pub = self.create_publisher(
                        Float64MultiArray, topics.pid_gains, 10
                    )
                    self.create_subscription(Twist, topics.cmd_vel, self._on_cmd, 10)
                    self.create_subscription(Odometry, topics.odom, self._on_odom, 10)
                    self.create_subscription(
                        Float64MultiArray, topics.pid, self._on_pid, 10
                    )

                def publish_twist(self, vx: float, vy: float, wz: float) -> None:
                    msg = Twist()
                    msg.linear.x = float(vx)
                    msg.linear.y = float(vy)
                    msg.angular.z = float(wz)
                    self.cmd_pub.publish(msg)

                def publish_gains(self, kp: float, ki: float, kd: float) -> None:
                    msg = Float64MultiArray()
                    msg.data = [float(kp), float(ki), float(kd)]
                    self.gains_pub.publish(msg)

                def _on_cmd(self, msg: Twist) -> None:
                    callbacks.on_cmd_vel(
                        float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)
                    )

                def _on_odom(self, msg: Odometry) -> None:
                    p = msg.pose.pose.position
                    o = msg.pose.pose.orientation
                    t = msg.twist.twist
                    yaw = quaternion_to_yaw(float(o.x), float(o.y), float(o.z), float(o.w))
                    callbacks.on_odom(
                        float(p.x),
                        float(p.y),
                        yaw,
                        float(t.linear.x),
                        float(t.linear.y),
                        float(t.angular.z),
                    )

                def _on_pid(self, msg: Float64MultiArray) -> None:
                    parsed_pid = parse_pid_array(msg.data)
                    if parsed_pid is not None:
                        callbacks.on_pid(*parsed_pid)

            node = _SubscriberNode()
            self._callbacks.on_status(
                ConnectionState.CONNECTED,
                f"Conectado ao ROS 2 (rclpy): {topics.cmd_vel}, {topics.odom}, {topics.pid}",
            )

            while not self._stop_event.is_set() and rclpy.ok():
                with self._cmd_lock:
                    pending_cmd = self._pending_cmd
                    self._pending_cmd = None
                    pending_gains = self._pending_gains
                    self._pending_gains = None
                if pending_cmd is not None:
                    node.publish_twist(*pending_cmd)
                if pending_gains is not None:
                    node.publish_gains(*pending_gains)
                rclpy.spin_once(node, timeout_sec=0.02)
        except Exception as exc:
            if not self._stop_event.is_set():
                self._callbacks.on_status(
                    ConnectionState.ERROR, f"Erro rclpy: {exc}"
                )
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if created_context:
                try:
                    if rclpy.ok():
                        rclpy.shutdown()
                except Exception:
                    pass
