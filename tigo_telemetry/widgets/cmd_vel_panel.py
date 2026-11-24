import math
from typing import Optional

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from core.models import CmdVelSample
from store.telemetry_store import TelemetryStore
from widgets.metric_card import MetricCard
from widgets.realtime_plot import PlotSeriesSpec, RealtimePlotWidget


class VectorIndicatorWidget(QWidget):
    """Indicador 2D de direção linear (vx, vy) e rotação (wz) sobre o chassi Mecanum."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFixedSize(156, 156)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

    def set_twist(self, vx: float, vy: float, wz: float) -> None:
        self._vx = vx
        self._vy = vy
        self._wz = wz
        self.update()

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        w = self.width()
        h = self.height()
        painter.fillRect(0, 0, w, h, QColor("#000000"))

        cx = w / 2.0
        cy = h / 2.0
        max_r = 58.0

        # Grade polar
        painter.setPen(QPen(QColor(70, 70, 70, 120), 1))
        for radius in (24.0, 48.0):
            painter.drawEllipse(QPointF(cx, cy), radius, radius)
        painter.drawLine(QPointF(cx - max_r, cy), QPointF(cx + max_r, cy))
        painter.drawLine(QPointF(cx, cy - max_r), QPointF(cx, cy + max_r))

        painter.setFont(QFont("Monospace", 8))
        painter.setPen(QColor("#64748b"))
        painter.drawText(QRectF(cx - 14, 2, 28, 14), Qt.AlignCenter, "+X")
        painter.drawText(QRectF(4, cy - 14, 20, 14), Qt.AlignLeft, "+Y")

        # Chassi Mecanum simplificado
        painter.setBrush(QColor(28, 28, 28, 230))
        painter.setPen(QPen(QColor("#64748b"), 1.5))
        painter.drawRect(QRectF(cx - 15, cy - 19, 30, 38))

        painter.setBrush(QColor("#94a3b8"))
        painter.setPen(Qt.NoPen)
        for wx, wy in ((-19, -16), (14, -16), (-19, 8), (14, 8)):
            painter.drawRect(QRectF(cx + wx, cy + wy, 5, 8))

        # Arco angular wz (em graus * 16 no Qt; anti-horário = positivo)
        if abs(self._wz) > 0.02:
            span_deg = min(155.0, abs(self._wz) * 35.0)
            start_deg = 90.0  # topo
            signed_span = span_deg if self._wz > 0 else -span_deg
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(QColor("#f59e0b"), 2.4))
            arc_rect = QRectF(cx - 38, cy - 38, 76, 76)
            painter.drawArc(arc_rect, int(start_deg * 16), int(signed_span * 16))

            end_rad = math.radians(start_deg + signed_span)
            tip_x = cx + 38 * math.cos(end_rad)
            tip_y = cy - 38 * math.sin(end_rad)
            tangent = -end_rad + (-math.pi / 2 if self._wz > 0 else math.pi / 2)
            self._draw_arrow_head(painter, tip_x, tip_y, tangent, "#f59e0b", 7.0)

        # Vetor linear (ROS: +X para cima, +Y para esquerda)
        speed = math.hypot(self._vx, self._vy)
        if speed > 0.01:
            ref_max = 0.8
            length = min(max_r, (speed / ref_max) * max_r)
            dx = -self._vy / speed
            dy = -self._vx / speed
            ex = cx + dx * length
            ey = cy + dy * length
            painter.setPen(QPen(QColor("#22d3ee"), 2.8))
            painter.drawLine(QPointF(cx, cy), QPointF(ex, ey))
            self._draw_arrow_head(painter, ex, ey, math.atan2(dy, dx), "#22d3ee", 8.5)

    @staticmethod
    def _draw_arrow_head(
        painter: QPainter,
        tip_x: float,
        tip_y: float,
        angle: float,
        color: str,
        size: float,
    ) -> None:
        p1 = QPointF(tip_x, tip_y)
        p2 = QPointF(
            tip_x - size * math.cos(angle - 0.42),
            tip_y - size * math.sin(angle - 0.42),
        )
        p3 = QPointF(
            tip_x - size * math.cos(angle + 0.42),
            tip_y - size * math.sin(angle + 0.42),
        )
        painter.setBrush(QColor(color))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(QPolygonF([p1, p2, p3]))


class CmdVelPanel(QFrame):
    """Painel completo para monitoramento de /cmd_vel (geometry_msgs/Twist)."""

    def __init__(self, store: TelemetryStore, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._store = store
        self.setObjectName("panelFrame")
        self.setStyleSheet(
            "#panelFrame { background-color: #0d0d0d; border: 1px solid #242424; border-radius: 10px; }"
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        header = QHBoxLayout()
        title = QLabel("Velocidade Comandada (/cmd_vel)")
        title.setStyleSheet("color: #f8fafc; font-size: 13px; font-weight: bold;")
        msg_type = QLabel("geometry_msgs/Twist")
        msg_type.setStyleSheet("color: #64748b; font-family: monospace; font-size: 11px;")
        header.addWidget(title)
        header.addStretch()
        header.addWidget(msg_type)
        root.addLayout(header)

        top_row = QHBoxLayout()
        top_row.setSpacing(10)

        cards_grid = QGridLayout()
        cards_grid.setSpacing(8)
        self._vx_card = MetricCard("linear.x", "m/s (Frente/Trás)", "#22d3ee")
        self._vy_card = MetricCard("linear.y", "m/s (Strafe)", "#34d399")
        self._wz_card = MetricCard("angular.z", "rad/s (Yaw)", "#f59e0b")
        cards_grid.addWidget(self._vx_card, 0, 0)
        cards_grid.addWidget(self._vy_card, 0, 1)
        cards_grid.addWidget(self._wz_card, 0, 2)
        top_row.addLayout(cards_grid, stretch=1)

        vec_box = QFrame()
        vec_box.setStyleSheet(
            "background-color: #000000; border: 1px solid #242424; border-radius: 8px;"
        )
        vec_layout = QVBoxLayout(vec_box)
        vec_layout.setContentsMargins(4, 4, 4, 4)
        self._vector_widget = VectorIndicatorWidget()
        vec_layout.addWidget(self._vector_widget)
        top_row.addWidget(vec_box)
        root.addLayout(top_row)

        self._plot = RealtimePlotWidget(
            [
                PlotSeriesSpec("vx", "#22d3ee", unit="m/s"),
                PlotSeriesSpec("vy", "#34d399", unit="m/s"),
                PlotSeriesSpec("wz", "#f59e0b", unit="rad/s"),
            ],
            min_height=205,
        )
        root.addWidget(self._plot, stretch=1)

    def refresh_charts(self, latest: Optional[CmdVelSample]) -> None:
        if latest is not None:
            self._vector_widget.set_twist(
                latest.linear_x, latest.linear_y, latest.angular_z
            )
        times, series = self._store.get_cmd_vel_series(15.0)
        self._plot.set_data(times, series)

    def refresh_numbers(self, latest: Optional[CmdVelSample]) -> None:
        if latest is None:
            self._vx_card.set_value("0.000")
            self._vy_card.set_value("0.000")
            self._wz_card.set_value("0.000")
        else:
            self._vx_card.set_value(f"{latest.linear_x:.3f}")
            self._vy_card.set_value(f"{latest.linear_y:.3f}")
            self._wz_card.set_value(f"{latest.angular_z:.3f}")
