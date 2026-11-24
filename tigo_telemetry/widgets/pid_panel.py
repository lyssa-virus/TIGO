from typing import Dict, Optional

from PyQt5.QtWidgets import (
    QButtonGroup,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from core.kinematics import DEFAULT_KD, DEFAULT_KI, DEFAULT_KP
from core.models import PidSample, WHEEL_IDS, WHEEL_LABELS
from store.telemetry_store import TelemetryStore
from widgets.metric_card import MetricCard
from widgets.realtime_plot import PlotSeriesSpec, RealtimePlotWidget


class PidAnalysisPanel(QFrame):
    """Painel de análise do controle PID por roda (FL, FR, RL, RR)."""

    def __init__(self, store: TelemetryStore, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._store = store
        self._selected_wheel = "FL"
        self._window_sec = 15.0

        self.setObjectName("pidPanel")
        self.setStyleSheet(
            "#pidPanel { background-color: #0d0d0d; border: 1px solid #242424; border-radius: 10px; }"
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # Cabeçalho + ganhos ativos + janela
        header = QHBoxLayout()
        title = QLabel("Análise do Controle PID por Roda (/tigo/pid_debug)")
        title.setStyleSheet("color: #f8fafc; font-size: 13px; font-weight: bold;")
        header.addWidget(title)

        self._gains_label = QLabel("Ganhos ativos: Kp=--  Ki=--  Kd=--")
        self._gains_label.setStyleSheet(
            "background: #000000; color: #38bdf8; border: 1px solid #242424; "
            "border-radius: 6px; padding: 4px 10px; font-family: monospace; font-size: 11px;"
        )
        header.addWidget(self._gains_label)
        header.addStretch()

        win_label = QLabel("Janela:")
        win_label.setStyleSheet("color: #94a3b8; font-size: 11px;")
        header.addWidget(win_label)

        self._win_group = QButtonGroup(self)
        self._win_group.setExclusive(True)
        for sec in (10, 15, 30):
            btn = QPushButton(f"{sec}s")
            btn.setCheckable(True)
            btn.setChecked(sec == 15)
            btn.setStyleSheet(
                "QPushButton { background: #000000; color: #94a3b8; border: 1px solid #363636; border-radius: 5px; padding: 3px 9px; font-family: monospace; font-size: 11px; }"
                "QPushButton:checked { background: #164e63; color: #67e8f9; border-color: #06b6d4; }"
            )
            btn.clicked.connect(lambda _chk, s=float(sec): self._on_window_changed(s))
            self._win_group.addButton(btn)
            header.addWidget(btn)

        root.addLayout(header)

        # Barra unificada de ajuste de ganhos PID (Mock, ROS 2 rclpy e Rosbridge)
        gains_bar = QFrame()
        gains_bar.setStyleSheet(
            "background: #000000; border: 1px solid #242424; border-radius: 8px;"
        )
        gains_layout = QHBoxLayout(gains_bar)
        gains_layout.setContentsMargins(10, 6, 10, 6)
        gains_layout.setSpacing(10)

        tune_lbl = QLabel("Ajustar Ganhos:")
        tune_lbl.setStyleSheet("color: #cbd5e1; font-size: 11px; font-weight: bold;")
        gains_layout.addWidget(tune_lbl)

        self._kp_spin = self._create_gain_control("Kp", DEFAULT_KP, 0.2, 2, 200.0, "#38bdf8", gains_layout)
        self._ki_spin = self._create_gain_control("Ki", DEFAULT_KI, 0.5, 2, 200.0, "#34d399", gains_layout)
        self._kd_spin = self._create_gain_control("Kd", DEFAULT_KD, 0.005, 3, 50.0, "#fbbf24", gains_layout)

        apply_btn = QPushButton("Aplicar Ganhos")
        apply_btn.clicked.connect(self._apply_gains)
        apply_btn.setStyleSheet(
            "QPushButton { background: #0e7490; color: #ffffff; border: 1px solid #22d3ee; border-radius: 6px; padding: 4px 12px; font-size: 11px; font-weight: bold; }"
            "QPushButton:hover { background: #0891b2; }"
        )
        gains_layout.addWidget(apply_btn)

        reset_gains_btn = QPushButton("Padrão TIGO")
        reset_gains_btn.clicked.connect(self._reset_default_gains)
        reset_gains_btn.setStyleSheet(
            "QPushButton { background: #0d0d0d; color: #94a3b8; border: 1px solid #363636; border-radius: 6px; padding: 4px 10px; font-size: 11px; }"
            "QPushButton:hover { color: #f8fafc; border-color: #64748b; }"
        )
        gains_layout.addWidget(reset_gains_btn)

        gains_layout.addStretch()
        self._sent_status_lbl = QLabel("")
        self._sent_status_lbl.setStyleSheet("color: #67e8f9; font-family: monospace; font-size: 11px;")
        gains_layout.addWidget(self._sent_status_lbl)

        root.addWidget(gains_bar)

        # Botões seletores das 4 rodas
        wheels_row = QHBoxLayout()
        wheels_row.setSpacing(8)
        self._wheel_group = QButtonGroup(self)
        self._wheel_group.setExclusive(True)
        self._wheel_buttons: Dict[str, QPushButton] = {}
        for wid in WHEEL_IDS:
            btn = QPushButton(f"{WHEEL_LABELS[wid]}\nSP: 0.00 | PV: 0.00 | PWM: 0")
            btn.setCheckable(True)
            btn.setChecked(wid == self._selected_wheel)
            btn.setStyleSheet(
                "QPushButton { text-align: left; background: #000000; color: #cbd5e1; border: 1px solid #242424; border-radius: 8px; padding: 8px 10px; font-family: monospace; font-size: 11px; }"
                "QPushButton:checked { background: #171717; color: #f8fafc; border-color: #06b6d4; }"
            )
            btn.clicked.connect(lambda _chk, w=wid: self._on_wheel_selected(w))
            self._wheel_group.addButton(btn)
            self._wheel_buttons[wid] = btn
            wheels_row.addWidget(btn)
        root.addLayout(wheels_row)

        # KPIs da roda selecionada
        kpi_grid = QGridLayout()
        kpi_grid.setSpacing(8)
        self._sp_card = MetricCard("Setpoint", "rad/s", "#38bdf8")
        self._pv_card = MetricCard("Medido", "rad/s", "#34d399")
        self._err_card = MetricCard("Erro Atual", "rad/s", "#fb7185")
        self._ff_card = MetricCard("Feedforward", "PWM", "#94a3b8")
        self._pwm_card = MetricCard("Saída Controle Total", "PWM", "#fbbf24")
        for idx, card in enumerate(
            (self._sp_card, self._pv_card, self._err_card, self._ff_card, self._pwm_card)
        ):
            kpi_grid.addWidget(card, 0, idx)
        root.addLayout(kpi_grid)

        # Gráficos lado a lado
        charts_row = QHBoxLayout()
        charts_row.setSpacing(10)

        self._tracking_plot = RealtimePlotWidget(
            [
                PlotSeriesSpec("Setpoint", "#38bdf8", dashed=True, unit="rad/s"),
                PlotSeriesSpec("Medido", "#34d399", unit="rad/s"),
            ],
            min_height=215,
        )
        self._effort_plot = RealtimePlotWidget(
            [
                PlotSeriesSpec("Erro", "#fb7185", unit="rad/s"),
                PlotSeriesSpec("PWM Total", "#fbbf24", right_axis=True),
                PlotSeriesSpec("Feedforward", "#94a3b8", dashed=True, right_axis=True),
            ],
            right_range=(-265.0, 265.0),
            min_height=215,
        )

        charts_row.addWidget(self._tracking_plot, stretch=1)
        charts_row.addWidget(self._effort_plot, stretch=1)
        root.addLayout(charts_row, stretch=1)

    def _create_gain_control(
        self,
        label: str,
        initial: float,
        step: float,
        decimals: int,
        max_val: float,
        color: str,
        parent_layout: QHBoxLayout,
    ) -> QDoubleSpinBox:
        lbl = QLabel(label)
        lbl.setStyleSheet(f"color: {color}; font-family: monospace; font-size: 11px; font-weight: bold;")

        minus_btn = QPushButton("-")
        minus_btn.setFixedSize(24, 24)
        minus_btn.setStyleSheet(
            "QPushButton { background: #0d0d0d; color: #cbd5e1; border: 1px solid #363636; border-radius: 4px; font-weight: bold; }"
            "QPushButton:hover { border-color: #64748b; color: #ffffff; }"
        )

        spin = QDoubleSpinBox()
        spin.setDecimals(decimals)
        spin.setRange(0.0, max_val)
        spin.setSingleStep(step)
        spin.setValue(initial)
        spin.setFixedWidth(72)
        spin.setStyleSheet(
            "background: #0d0d0d; color: #f8fafc; border: 1px solid #363636; "
            "border-radius: 4px; padding: 2px 5px; font-family: monospace; font-size: 11px;"
        )

        plus_btn = QPushButton("+")
        plus_btn.setFixedSize(24, 24)
        plus_btn.setStyleSheet(minus_btn.styleSheet())

        minus_btn.clicked.connect(lambda: self._step_gain(spin, -1))
        plus_btn.clicked.connect(lambda: self._step_gain(spin, 1))

        parent_layout.addWidget(lbl)
        parent_layout.addWidget(minus_btn)
        parent_layout.addWidget(spin)
        parent_layout.addWidget(plus_btn)
        return spin

    def _step_gain(self, spin: QDoubleSpinBox, direction: int) -> None:
        if direction > 0:
            spin.stepUp()
        else:
            spin.stepDown()
        self._apply_gains()

    def _apply_gains(self) -> None:
        kp = self._kp_spin.value()
        ki = self._ki_spin.value()
        kd = self._kd_spin.value()
        self._store.send_pid_gains(kp, ki, kd)
        self._sent_status_lbl.setText(
            f"Enviado: Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.3f}"
        )

    def _reset_default_gains(self) -> None:
        self._kp_spin.setValue(DEFAULT_KP)
        self._ki_spin.setValue(DEFAULT_KI)
        self._kd_spin.setValue(DEFAULT_KD)
        self._apply_gains()

    def _on_wheel_selected(self, wheel: str) -> None:
        self._selected_wheel = wheel
        self.refresh_charts()
        self.refresh_numbers(self._store.get_snapshot().pid)

    def _on_window_changed(self, window_sec: float) -> None:
        self._window_sec = window_sec
        self.refresh_charts()

    def refresh_charts(self) -> None:
        t_trk, s_trk = self._store.get_pid_tracking_series(
            self._selected_wheel, self._window_sec
        )
        self._tracking_plot.set_data(t_trk, s_trk)

        t_eff, s_eff = self._store.get_pid_effort_series(
            self._selected_wheel, self._window_sec
        )
        self._effort_plot.set_data(t_eff, s_eff)

    def refresh_numbers(self, pid: Optional[PidSample]) -> None:
        if pid is None:
            self._gains_label.setText("Ganhos ativos: Kp=--  Ki=--  Kd=--")
            for card in (
                self._sp_card,
                self._pv_card,
                self._err_card,
                self._ff_card,
                self._pwm_card,
            ):
                card.set_value("--")
            return

        self._gains_label.setText(
            f"Ganhos ativos: Kp={pid.kp:.2f}  Ki={pid.ki:.2f}  Kd={pid.kd:.3f}"
        )

        for wid in WHEEL_IDS:
            w = pid.wheels[wid]
            self._wheel_buttons[wid].setText(
                f"{WHEEL_LABELS[wid]}\nSP: {w.setpoint:.2f} | PV: {w.measured:.2f} | PWM: {w.total_pwm:.0f}"
            )

        active = pid.wheels[self._selected_wheel]
        self._sp_card.set_value(f"{active.setpoint:.2f}")
        self._pv_card.set_value(f"{active.measured:.2f}")
        self._err_card.set_value(f"{active.error:.2f}")
        self._ff_card.set_value(f"{active.ff_pwm:.1f}")
        self._pwm_card.set_value(f"{active.total_pwm:.1f}")
