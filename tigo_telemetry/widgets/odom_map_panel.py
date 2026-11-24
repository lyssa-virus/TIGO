import math
from typing import List, Optional, Tuple

from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt
from PyQt5.QtGui import (
    QColor,
    QFont,
    QMouseEvent,
    QPainter,
    QPainterPath,
    QPen,
    QWheelEvent,
)
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from core.models import OdomSample
from store.telemetry_store import TelemetryStore
from widgets.metric_card import MetricCard


class OdomMapWidget(QWidget):
    """Mapa 2D interativo para trajetória de odometria (/odom) com pan e zoom."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(235)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setCursor(Qt.OpenHandCursor)

        self._trail: List[Tuple[float, float]] = []
        self._ideal_trail: List[Tuple[float, float]] = []
        self._pose: Optional[OdomSample] = None
        self._ideal_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._center_x = 0.0
        self._center_y = 0.0
        self._px_per_m = 90.0
        self._follow_robot = True
        self._dragging = False
        self._last_pos = QPoint()

    def set_follow_robot(self, enabled: bool) -> None:
        self._follow_robot = enabled
        self.update()

    def set_state(
        self,
        trail: List[Tuple[float, float]],
        ideal_trail: List[Tuple[float, float]],
        pose: Optional[OdomSample],
        ideal_pose: Tuple[float, float, float],
    ) -> None:
        self._trail = trail
        self._ideal_trail = ideal_trail
        self._pose = pose
        self._ideal_pose = ideal_pose
        if self._follow_robot and pose is not None:
            self._center_x = pose.x
            self._center_y = pose.y
        self.update()

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._last_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if not self._dragging:
            return
        dx = event.x() - self._last_pos.x()
        dy = event.y() - self._last_pos.y()
        self._last_pos = event.pos()

        self._follow_robot = False
        self._center_x -= dx / self._px_per_m
        self._center_y += dy / self._px_per_m
        self.update()

    def mouseReleaseEvent(self, _event: QMouseEvent) -> None:
        self._dragging = False
        self.setCursor(Qt.OpenHandCursor)

    def wheelEvent(self, event: QWheelEvent) -> None:
        factor = 1.12 if event.angleDelta().y() > 0 else 0.89
        self._px_per_m = max(20.0, min(320.0, self._px_per_m * factor))
        self.update()

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        w = self.width()
        h = self.height()
        painter.fillRect(0, 0, w, h, QColor("#000000"))

        cx = w / 2.0
        cy = h / 2.0
        scale = self._px_per_m

        def to_screen(wx: float, wy: float) -> Tuple[float, float]:
            return cx + (wx - self._center_x) * scale, cy - (wy - self._center_y) * scale

        step_m = 1.0 if scale < 45.0 else 0.5
        min_x = math.floor((self._center_x - cx / scale) / step_m) * step_m
        max_x = math.ceil((self._center_x + cx / scale) / step_m) * step_m
        min_y = math.floor((self._center_y - cy / scale) / step_m) * step_m
        max_y = math.ceil((self._center_y + cy / scale) / step_m) * step_m

        painter.setFont(QFont("Monospace", 8))

        gx = min_x
        while gx <= max_x + 1e-6:
            sx, _ = to_screen(gx, 0.0)
            color = (
                QColor(244, 63, 94, 95) if abs(gx) < 1e-5 else QColor(50, 50, 50, 125)
            )
            painter.setPen(QPen(color, 1))
            painter.drawLine(QPointF(sx, 0), QPointF(sx, h))
            if abs(gx % 1.0) < 1e-5:
                painter.setPen(QColor("#64748b"))
                painter.drawText(QPointF(sx + 3, h - 6), f"{gx:.0f}m")
            gx += step_m

        gy = min_y
        while gy <= max_y + 1e-6:
            _, sy = to_screen(0.0, gy)
            color = (
                QColor(34, 197, 94, 95) if abs(gy) < 1e-5 else QColor(50, 50, 50, 125)
            )
            painter.setPen(QPen(color, 1))
            painter.drawLine(QPointF(0, sy), QPointF(w, sy))
            if abs(gy % 1.0) < 1e-5:
                painter.setPen(QColor("#64748b"))
                painter.drawText(QPointF(6, sy - 4), f"{gy:.0f}m")
            gy += step_m

        # Origem (0, 0)
        ox, oy = to_screen(0.0, 0.0)
        painter.setPen(QPen(QColor(148, 163, 184, 150), 1.2))
        painter.drawLine(QPointF(ox - 6, oy), QPointF(ox + 6, oy))
        painter.drawLine(QPointF(ox, oy - 6), QPointF(ox, oy + 6))

        # Trajetória ideal (azul - /cmd_vel) e medida (vermelha - /odom)
        self._draw_trail(painter, self._ideal_trail, to_screen, "#3b82f6", 2.2)
        self._draw_trail(painter, self._trail, to_screen, "#ef4444", 2.2)

        # Marcador da pose ideal (/cmd_vel)
        if self._ideal_trail:
            isx, isy = to_screen(self._ideal_pose[0], self._ideal_pose[1])
            painter.setBrush(QColor(59, 130, 246, 55))
            painter.setPen(QPen(QColor("#3b82f6"), 1.6, Qt.DashLine))
            painter.drawEllipse(QPointF(isx, isy), 8.0, 8.0)

        # Chassi na pose medida (/odom)
        rx = self._pose.x if self._pose else 0.0
        ry = self._pose.y if self._pose else 0.0
        ryaw = self._pose.yaw if self._pose else 0.0
        rsx, rsy = to_screen(rx, ry)

        painter.save()
        painter.translate(rsx, rsy)
        # QPainter tem +Y para baixo: girar -degrees(yaw) alinha +X ao yaw do mundo
        painter.rotate(-math.degrees(ryaw))

        half_len = max(10.0, 0.14 * scale)
        half_wid = max(8.0, 0.11 * scale)
        painter.setBrush(QColor(239, 68, 68, 65))
        painter.setPen(QPen(QColor("#ef4444"), 2.0))
        painter.drawRect(QRectF(-half_len, -half_wid, half_len * 2, half_wid * 2))

        painter.setPen(QPen(QColor("#facc15"), 2.5))
        painter.drawLine(QPointF(0, 0), QPointF(half_len + 10, 0))
        painter.restore()

        # Legenda das duas trajetórias
        painter.setBrush(QColor(0, 0, 0, 200))
        painter.setPen(QPen(QColor("#242424"), 1.0))
        painter.drawRoundedRect(QRectF(8, 8, 178, 40), 6, 6)

        painter.setPen(QPen(QColor("#3b82f6"), 2.5))
        painter.drawLine(QPointF(16, 20), QPointF(32, 20))
        painter.setPen(QColor("#e2e8f0"))
        painter.drawText(QPointF(38, 24), "Ideal (/cmd_vel)")

        painter.setPen(QPen(QColor("#ef4444"), 2.5))
        painter.drawLine(QPointF(16, 37), QPointF(32, 37))
        painter.setPen(QColor("#e2e8f0"))
        painter.drawText(QPointF(38, 41), "Medida (/odom)")

        painter.setPen(QColor("#64748b"))
        painter.drawText(
            QRectF(8, h - 18, w - 16, 14),
            Qt.AlignRight,
            "Arraste para mover • Scroll para zoom",
        )

    @staticmethod
    def _draw_trail(
        painter: QPainter,
        trail: List[Tuple[float, float]],
        to_screen,
        color: str,
        width: float,
    ) -> None:
        if len(trail) < 2:
            return
        path = QPainterPath()
        x0, y0 = to_screen(trail[0][0], trail[0][1])
        path.moveTo(x0, y0)
        for px, py in trail[1:]:
            sx, sy = to_screen(px, py)
            path.lineTo(sx, sy)
        painter.setPen(QPen(QColor(color), width))
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(path)


class OdomMapPanel(QFrame):
    """Painel de Odometria (/odom) com pose, velocidades estimadas e mapa 2D."""

    def __init__(self, store: TelemetryStore, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._store = store
        self.setObjectName("odomPanel")
        self.setStyleSheet(
            "#odomPanel { background-color: #0d0d0d; border: 1px solid #242424; border-radius: 10px; }"
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        header = QHBoxLayout()
        title = QLabel("Odometria & Trajetória 2D (/odom)")
        title.setStyleSheet("color: #f8fafc; font-size: 13px; font-weight: bold;")
        header.addWidget(title)
        header.addStretch()

        self._follow_btn = QPushButton("Seguir Robô")
        self._follow_btn.setCheckable(True)
        self._follow_btn.setChecked(True)
        self._follow_btn.clicked.connect(self._on_follow_toggled)
        self._follow_btn.setStyleSheet(
            "QPushButton { background: #000000; color: #cbd5e1; border: 1px solid #363636; border-radius: 6px; padding: 4px 10px; font-size: 11px; }"
            "QPushButton:checked { background: #064e3b; color: #6ee7b7; border-color: #10b981; }"
        )
        header.addWidget(self._follow_btn)

        clear_btn = QPushButton("Limpar Rastro")
        clear_btn.clicked.connect(self._on_clear_trail)
        clear_btn.setStyleSheet(
            "QPushButton { background: #000000; color: #cbd5e1; border: 1px solid #363636; border-radius: 6px; padding: 4px 10px; font-size: 11px; }"
            "QPushButton:hover { border-color: #f43f5e; color: #fda4af; }"
        )
        header.addWidget(clear_btn)
        root.addLayout(header)

        grid = QGridLayout()
        grid.setSpacing(6)
        self._x_card = MetricCard("Pos X (m)", "", "#34d399")
        self._y_card = MetricCard("Pos Y (m)", "", "#34d399")
        self._yaw_card = MetricCard("Yaw (°)", "", "#facc15")
        self._vx_card = MetricCard("vx real", "", "#67e8f9")
        self._vy_card = MetricCard("vy real", "", "#67e8f9")
        self._wz_card = MetricCard("wz real", "", "#fcd34d")

        for idx, card in enumerate(
            (
                self._x_card,
                self._y_card,
                self._yaw_card,
                self._vx_card,
                self._vy_card,
                self._wz_card,
            )
        ):
            grid.addWidget(card, 0, idx)
        root.addLayout(grid)

        map_frame = QFrame()
        map_frame.setStyleSheet("border: 1px solid #242424; border-radius: 8px;")
        map_layout = QVBoxLayout(map_frame)
        map_layout.setContentsMargins(1, 1, 1, 1)
        self._map_widget = OdomMapWidget()
        map_layout.addWidget(self._map_widget)
        root.addWidget(map_frame, stretch=1)

    def _on_follow_toggled(self, checked: bool) -> None:
        self._map_widget.set_follow_robot(checked)

    def _on_clear_trail(self) -> None:
        self._store.clear_trail()
        self.refresh_map()

    def refresh_map(self) -> None:
        trail, ideal_trail, pose, ideal_pose = self._store.get_odom_trail()
        self._map_widget.set_state(trail, ideal_trail, pose, ideal_pose)

    def refresh_numbers(self, latest: Optional[OdomSample]) -> None:
        if latest is None:
            for card in (self._x_card, self._y_card, self._vx_card, self._vy_card, self._wz_card):
                card.set_value("0.000")
            self._yaw_card.set_value("0.0°")
        else:
            self._x_card.set_value(f"{latest.x:.3f}")
            self._y_card.set_value(f"{latest.y:.3f}")
            self._yaw_card.set_value(f"{math.degrees(latest.yaw):.1f}°")
            self._vx_card.set_value(f"{latest.vx:.3f}")
            self._vy_card.set_value(f"{latest.vy:.3f}")
            self._wz_card.set_value(f"{latest.wz:.3f}")
