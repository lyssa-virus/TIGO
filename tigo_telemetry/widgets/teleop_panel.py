from typing import Optional, Tuple

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from store.telemetry_store import TelemetryStore


class TeleopPanel(QFrame):
    """
    Painel de controle manual (teleop) por botões para o robô omnidirecional TIGO.
    Publica comandos em /cmd_vel nos modos Mock, ROS 2 (rclpy) e Rosbridge.
    """

    command_triggered = pyqtSignal()

    def __init__(self, store: TelemetryStore, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._store = store
        self._active_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        self.setObjectName("teleopPanel")
        self.setStyleSheet(
            "#teleopPanel { background-color: #0d0d0d; border: 1px solid #242424; border-radius: 10px; }"
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # Cabeçalho
        header = QHBoxLayout()
        title = QLabel("Controle Manual (Teleop /cmd_vel)")
        title.setStyleSheet("color: #f8fafc; font-size: 13px; font-weight: bold;")
        header.addWidget(title)
        header.addStretch()

        self._cmd_status_lbl = QLabel("Parado (0.00, 0.00, 0.00)")
        self._cmd_status_lbl.setStyleSheet(
            "background: #000000; color: #94a3b8; border: 1px solid #242424; "
            "border-radius: 5px; padding: 3px 8px; font-family: monospace; font-size: 11px;"
        )
        header.addWidget(self._cmd_status_lbl)
        root.addLayout(header)

        # Ajustes de velocidade e trava
        controls_row = QHBoxLayout()
        controls_row.setSpacing(10)

        self._lin_spin = self._create_speed_spin("Vel. Linear (m/s):", 0.35, 0.05, 1.00, 0.05, controls_row)
        self._ang_spin = self._create_speed_spin("Vel. Angular (rad/s):", 1.00, 0.10, 2.50, 0.10, controls_row)

        self._latch_check = QCheckBox("Manter ao clicar")
        self._latch_check.setToolTip(
            "Desmarcado: move apenas enquanto segura o botão.\n"
            "Marcado: mantém o comando ativo até clicar em PARAR."
        )
        self._latch_check.setStyleSheet("color: #cbd5e1; font-size: 11px;")
        controls_row.addWidget(self._latch_check)
        controls_row.addStretch()
        root.addLayout(controls_row)

        # Grade 3x3 com os 6 movimentos solicitados + PARAR
        pad = QGridLayout()
        pad.setSpacing(8)

        btn_style_move = (
            "QPushButton { background: #000000; color: #e2e8f0; border: 1px solid #363636; "
            "border-radius: 8px; padding: 10px 8px; font-size: 12px; font-weight: bold; }"
            "QPushButton:hover { border-color: #22d3ee; background: #171717; color: #ffffff; }"
            "QPushButton:pressed { background: #0e7490; border-color: #67e8f9; }"
        )
        btn_style_rot = (
            "QPushButton { background: #000000; color: #fde68a; border: 1px solid #363636; "
            "border-radius: 8px; padding: 10px 8px; font-size: 12px; font-weight: bold; }"
            "QPushButton:hover { border-color: #f59e0b; background: #451a03; color: #ffffff; }"
            "QPushButton:pressed { background: #b45309; border-color: #fcd34d; }"
        )
        btn_style_stop = (
            "QPushButton { background: #4c0519; color: #fecdd3; border: 1px solid #e11d48; "
            "border-radius: 8px; padding: 10px 8px; font-size: 12px; font-weight: bold; }"
            "QPushButton:hover { background: #9f1239; color: #ffffff; }"
            "QPushButton:pressed { background: #e11d48; }"
        )

        # Definição explícita (linha, coluna, texto, estilo, eixo_vx, eixo_vy, eixo_wz)
        directions = [
            (0, 0, "↺ Girar Esq.\n(+ωz)", btn_style_rot, 0.0, 0.0, 1.0),
            (0, 1, "↑ Frente\n(+vx)", btn_style_move, 1.0, 0.0, 0.0),
            (0, 2, "↻ Girar Dir.\n(-ωz)", btn_style_rot, 0.0, 0.0, -1.0),
            (1, 0, "← Lat. Esq.\n(+vy)", btn_style_move, 0.0, 1.0, 0.0),
            (1, 2, "→ Lat. Dir.\n(-vy)", btn_style_move, 0.0, -1.0, 0.0),
            (2, 1, "↓ Trás\n(-vx)", btn_style_move, -1.0, 0.0, 0.0),
        ]

        for row, col, label, style, sx, sy, sz in directions:
            btn = QPushButton(label)
            btn.setStyleSheet(style)
            btn.setMinimumHeight(48)
            btn.pressed.connect(
                lambda x=sx, y=sy, z=sz: self._on_dir_pressed(x, y, z)
            )
            btn.released.connect(self._on_dir_released)
            pad.addWidget(btn, row, col)

        stop_btn = QPushButton("■ PARAR\n(0, 0, 0)")
        stop_btn.setStyleSheet(btn_style_stop)
        stop_btn.setMinimumHeight(48)
        stop_btn.clicked.connect(self.stop_robot)
        pad.addWidget(stop_btn, 1, 1)

        hint = QLabel("Dica: segure o botão para mover\nou marque 'Manter ao clicar'.")
        hint.setAlignment(Qt.AlignCenter)
        hint.setStyleSheet("color: #64748b; font-size: 10px;")
        pad.addWidget(hint, 2, 0)

        hint_axes = QLabel("Padrão ROS:\n+X frente | +Y esq | +Z anti-hor.")
        hint_axes.setAlignment(Qt.AlignCenter)
        hint_axes.setStyleSheet("color: #64748b; font-family: monospace; font-size: 10px;")
        pad.addWidget(hint_axes, 2, 2)

        root.addLayout(pad)

    @staticmethod
    def _create_speed_spin(
        label_text: str,
        initial: float,
        min_v: float,
        max_v: float,
        step: float,
        parent_layout: QHBoxLayout,
    ) -> QDoubleSpinBox:
        lbl = QLabel(label_text)
        lbl.setStyleSheet("color: #94a3b8; font-size: 11px;")
        spin = QDoubleSpinBox()
        spin.setDecimals(2)
        spin.setRange(min_v, max_v)
        spin.setSingleStep(step)
        spin.setValue(initial)
        spin.setFixedWidth(68)
        spin.setStyleSheet(
            "background: #000000; color: #f8fafc; border: 1px solid #363636; "
            "border-radius: 5px; padding: 3px 5px; font-family: monospace; font-size: 11px;"
        )
        parent_layout.addWidget(lbl)
        parent_layout.addWidget(spin)
        return spin

    def _on_dir_pressed(self, sx: float, sy: float, sz: float) -> None:
        v_lin = self._lin_spin.value()
        v_ang = self._ang_spin.value()
        self._apply_cmd(sx * v_lin, sy * v_lin, sz * v_ang)

    def _on_dir_released(self) -> None:
        if not self._latch_check.isChecked():
            self.stop_robot()

    def stop_robot(self) -> None:
        self._apply_cmd(0.0, 0.0, 0.0)

    def _apply_cmd(self, vx: float, vy: float, wz: float) -> None:
        self._active_cmd = (vx, vy, wz)
        self._store.send_teleop_cmd(vx, vy, wz)
        self.command_triggered.emit()
        moving = abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4
        if moving:
            self._cmd_status_lbl.setText(f"Ativo ({vx:+.2f}, {vy:+.2f}, {wz:+.2f})")
            self._cmd_status_lbl.setStyleSheet(
                "background: #141414; color: #67e8f9; border: 1px solid #06b6d4; "
                "border-radius: 5px; padding: 3px 8px; font-family: monospace; font-size: 11px;"
            )
        else:
            self._cmd_status_lbl.setText("Parado (0.00, 0.00, 0.00)")
            self._cmd_status_lbl.setStyleSheet(
                "background: #000000; color: #94a3b8; border: 1px solid #242424; "
                "border-radius: 5px; padding: 3px 8px; font-family: monospace; font-size: 11px;"
            )
