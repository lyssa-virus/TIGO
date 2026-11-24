import base64
import hashlib
import json
import os
import socket
import struct
import threading
from typing import Any, Dict, Optional, Tuple

from core.kinematics import quaternion_to_yaw
from core.models import ConnectionConfig, ConnectionState
from providers.base import TelemetryCallbacks, TelemetryProvider, parse_pid_array

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def encode_ws_frame(payload: bytes, opcode: int = 0x1) -> bytes:
    """Codifica um frame WebSocket RFC 6455 mascarado (cliente -> servidor)."""
    mask_key = os.urandom(4)
    length = len(payload)
    header = bytearray([0x80 | (opcode & 0x0F)])

    if length < 126:
        header.append(0x80 | length)
    elif length < 65536:
        header.append(0x80 | 126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(0x80 | 127)
        header.extend(struct.pack("!Q", length))

    header.extend(mask_key)
    masked = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
    return bytes(header) + masked


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < n:
        chunk = sock.recv(n - len(chunks))
        if not chunk:
            raise ConnectionError("Conexão encerrada pelo servidor")
        chunks.extend(chunk)
    return bytes(chunks)


def decode_ws_frame(sock: socket.socket) -> Tuple[bool, int, bytes]:
    """Lê um frame WebSocket RFC 6455 enviado pelo servidor."""
    b1, b2 = _recv_exact(sock, 2)
    fin = bool(b1 & 0x80)
    opcode = b1 & 0x0F
    masked = bool(b2 & 0x80)
    length = b2 & 0x7F

    if length == 126:
        (length,) = struct.unpack("!H", _recv_exact(sock, 2))
    elif length == 127:
        (length,) = struct.unpack("!Q", _recv_exact(sock, 8))

    mask_key = _recv_exact(sock, 4) if masked else b""
    payload = _recv_exact(sock, length) if length > 0 else b""
    if masked:
        payload = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
    return fin, opcode, payload


class RosbridgeWsClient(TelemetryProvider):
    """
    Cliente WebSocket puro Python (RFC 6455) para o protocolo rosbridge_server v2.
    Dispensa bibliotecas externas e suporta reconexão automática.
    """

    def __init__(self, config: ConnectionConfig, callbacks: TelemetryCallbacks) -> None:
        self._config = config
        self._callbacks = callbacks
        self._stop_event = threading.Event()
        self._send_lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._close_socket()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None
        self._callbacks.on_status(ConnectionState.DISCONNECTED, "Desconectado")

    def _close_socket(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _perform_handshake(self, sock: socket.socket, host: str, port: int) -> None:
        ws_key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            f"GET / HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {ws_key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        sock.sendall(request.encode("ascii"))

        buf = bytearray()
        while b"\r\n\r\n" not in buf:
            chunk = sock.recv(1024)
            if not chunk:
                raise ConnectionError("Handshake WebSocket encerrado sem resposta")
            buf.extend(chunk)
            if len(buf) > 16384:
                raise ConnectionError("Cabeçalho HTTP muito grande")

        header_text = buf.split(b"\r\n\r\n", 1)[0].decode("latin1")
        if " 101 " not in header_text.splitlines()[0]:
            raise ConnectionError(f"Handshake recusado: {header_text.splitlines()[0]}")

        expected_accept = base64.b64encode(
            hashlib.sha1((ws_key + _WS_GUID).encode("ascii")).digest()
        ).decode("ascii")
        if expected_accept not in header_text:
            raise ConnectionError("Sec-WebSocket-Accept inválido")

    def _send_json(self, sock: socket.socket, payload: Dict[str, Any]) -> None:
        raw = json.dumps(payload).encode("utf-8")
        with self._send_lock:
            sock.sendall(encode_ws_frame(raw, opcode=0x1))

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        sock = self._sock
        topic = self._config.topics.cmd_vel.strip()
        if sock is None or not topic:
            return
        try:
            self._send_json(
                sock,
                {
                    "op": "publish",
                    "topic": topic,
                    "msg": {
                        "linear": {"x": float(vx), "y": float(vy), "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": float(wz)},
                    },
                },
            )
        except OSError:
            pass

    def send_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        sock = self._sock
        topic = self._config.topics.pid_gains.strip()
        if sock is None or not topic:
            return
        try:
            self._send_json(
                sock,
                {
                    "op": "publish",
                    "topic": topic,
                    "msg": {"data": [float(kp), float(ki), float(kd)]},
                },
            )
        except OSError:
            pass

    def _subscribe_all(self, sock: socket.socket) -> None:
        topics = self._config.topics
        for topic, msg_type in (
            (topics.cmd_vel, "geometry_msgs/msg/Twist"),
            (topics.pid_gains, "std_msgs/msg/Float64MultiArray"),
        ):
            if topic.strip():
                self._send_json(
                    sock,
                    {
                        "op": "advertise",
                        "topic": topic,
                        "type": msg_type,
                    },
                )
        subs = [
            (topics.cmd_vel, "geometry_msgs/msg/Twist"),
            (topics.odom, "nav_msgs/msg/Odometry"),
            (topics.pid, "std_msgs/msg/Float64MultiArray"),
        ]
        for topic, msg_type in subs:
            if topic.strip():
                self._send_json(
                    sock,
                    {
                        "op": "subscribe",
                        "topic": topic,
                        "type": msg_type,
                        "queue_length": 1,
                    },
                )

    def _worker_loop(self) -> None:
        host = self._config.host
        port = self._config.port
        url = f"ws://{host}:{port}"

        while not self._stop_event.is_set():
            sock: Optional[socket.socket] = None
            try:
                self._callbacks.on_status(
                    ConnectionState.CONNECTING, f"Conectando a {url}..."
                )
                sock = socket.create_connection((host, port), timeout=3.0)
                sock.settimeout(2.0)
                self._sock = sock
                self._perform_handshake(sock, host, port)
                self._subscribe_all(sock)

                self._callbacks.on_status(
                    ConnectionState.CONNECTED, f"Conectado em {url}"
                )

                fragmented = bytearray()
                while not self._stop_event.is_set():
                    try:
                        fin, opcode, payload = decode_ws_frame(sock)
                    except socket.timeout:
                        continue

                    if opcode == 0x8:  # Close
                        break
                    if opcode == 0x9:  # Ping -> Pong
                        with self._send_lock:
                            sock.sendall(encode_ws_frame(payload, opcode=0xA))
                        continue
                    if opcode in (0x1, 0x0):  # Text ou Continuação
                        fragmented.extend(payload)
                        if fin:
                            text = fragmented.decode("utf-8", errors="replace")
                            fragmented.clear()
                            self._handle_message(text)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._callbacks.on_status(
                        ConnectionState.ERROR,
                        f"Sem conexão com {url} ({exc}). Reconectando...",
                    )
            finally:
                self._close_socket()

            if not self._stop_event.is_set():
                self._stop_event.wait(2.5)

    def _handle_message(self, raw: str) -> None:
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return

        if parsed.get("op") != "publish":
            return
        topic = parsed.get("topic")
        msg = parsed.get("msg")
        if not isinstance(msg, dict):
            return

        topics = self._config.topics
        if topic == topics.cmd_vel:
            linear = msg.get("linear") or {}
            angular = msg.get("angular") or {}
            self._callbacks.on_cmd_vel(
                float(linear.get("x", 0.0)),
                float(linear.get("y", 0.0)),
                float(angular.get("z", 0.0)),
            )
        elif topic == topics.odom:
            pose = (msg.get("pose") or {}).get("pose") or {}
            pos = pose.get("position") or {}
            ori = pose.get("orientation") or {}
            twist = (msg.get("twist") or {}).get("twist") or {}
            t_lin = twist.get("linear") or {}
            t_ang = twist.get("angular") or {}
            yaw = quaternion_to_yaw(
                float(ori.get("x", 0.0)),
                float(ori.get("y", 0.0)),
                float(ori.get("z", 0.0)),
                float(ori.get("w", 1.0)),
            )
            self._callbacks.on_odom(
                float(pos.get("x", 0.0)),
                float(pos.get("y", 0.0)),
                yaw,
                float(t_lin.get("x", 0.0)),
                float(t_lin.get("y", 0.0)),
                float(t_ang.get("z", 0.0)),
            )
        elif topic == topics.pid:
            data = msg.get("data")
            if not isinstance(data, list):
                return
            parsed_pid = parse_pid_array(data)
            if parsed_pid is not None:
                self._callbacks.on_pid(*parsed_pid)
