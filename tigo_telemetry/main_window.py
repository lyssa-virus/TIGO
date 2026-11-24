from datetime import datetime
from typing import Dict, Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from core.models import (
    ConnectionConfig,
    ConnectionState,
    MOCK_SCENARIO_LABELS,
    MockScenario,
    TelemetryMode,
    TelemetrySnapshot,
    TopicNamesConfig,
)
from providers.ros2_native import is_rclpy_available
from store.telemetry_store import TelemetryStore
from widgets.cmd_vel_panel import CmdVelPanel
from widgets.odom_map_panel import OdomMapPanel
from widgets.pid_panel import PidAnalysisPanel
from widgets.teleop_panel import TeleopPanel

_STATUS_STYLES: Dict[ConnectionState, tuple[str, str]] = {
    ConnectionState.MOCK: ("Simulação (Mock)", "#141414; color: #67e8f9; border: 1px solid #06b6d4"),
    ConnectionState.CONNECTED: ("Conectado", "#064e3b; color: #6ee7b7; border: 1px solid #10b981"),
    ConnectionState.CONNECTING: ("Conectando...", "#451a03; color: #fcd34d; border: 1px solid #f59e0b"),
    ConnectionState.ERROR: ("Erro de Conexão", "#4c0519; color: #fda4af; border: 1px solid #f43f5e"),
    ConnectionState.DISCONNECTED: ("Desconectado", "#242424; color: #cbd5e1; border: 1px solid #475569"),
}


class MainWindow(QMainWindow):
    """Janela principal do aplicativo desktop de monitoramento do robô TIGO."""

    def __init__(self, store: Optional[TelemetryStore] = None) -> None:
        super().__init__()
        self._store = store or TelemetryStore()
        self._store.start()

        self.setWindowTitle("TIGO Telemetry — Monitoramento ROS 2 & PID")
        self.resize(1380, 900)
        self.setStyleSheet("QMainWindow { background-color: #000000; }")

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        main_layout.addWidget(self._build_header())
        main_layout.addWidget(self._build_topics_drawer())
        main_layout.addWidget(self._build_rates_and_mock_bar())

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("QScrollArea { background-color: #000000; }")

        content = QWidget()
        content.setStyleSheet("background-color: #000000;")
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(12, 12, 12, 12)
        content_layout.setSpacing(12)

        top_panels = QHBoxLayout()
        top_panels.setSpacing(12)
        self._cmd_panel = CmdVelPanel(self._store)
        self._odom_panel = OdomMapPanel(self._store)
        top_panels.addWidget(self._cmd_panel, stretch=1)
        top_panels.addWidget(self._odom_panel, stretch=1)
        content_layout.addLayout(top_panels)

        bottom_panels = QHBoxLayout()
        bottom_panels.setSpacing(12)
        self._teleop_panel = TeleopPanel(self._store)
        self._teleop_panel.setFixedWidth(380)
        self._teleop_panel.command_triggered.connect(self._on_teleop_triggered)
        self._pid_panel = PidAnalysisPanel(self._store)
        bottom_panels.addWidget(self._teleop_panel)
        bottom_panels.addWidget(self._pid_panel, stretch=1)
        content_layout.addLayout(bottom_panels)

        scroll.setWidget(content)
        main_layout.addWidget(scroll, stretch=1)

        # Timers desacoplados: 25 FPS para gráficos/mapa, 10 Hz para números e 20 Hz keepalive teleop
        self._chart_timer = QTimer(self)
        self._chart_timer.timeout.connect(self._on_chart_tick)
        self._chart_timer.start(40)

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._on_ui_tick)
        self._ui_timer.start(100)

        self._teleop_timer = QTimer(self)
        self._teleop_timer.timeout.connect(self._store.tick_teleop_keepalive)
        self._teleop_timer.start(50)

        self._on_ui_tick()

    def _build_header(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet("background-color: #0d0d0d; border-bottom: 1px solid #242424;")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(10)

        title = QLabel("TIGO Telemetry")
        title.setStyleSheet("color: #f8fafc; font-size: 15px; font-weight: bold;")
        layout.addWidget(title)

        self._status_badge = QLabel("Simulação (Mock)")
        layout.addWidget(self._status_badge)

        layout.addStretch()

        # Seletor de modo: Mock | ROS 2 Nativo | Rosbridge
        mode_group = QButtonGroup(self)
        mode_group.setExclusive(True)
        self._mode_buttons: Dict[TelemetryMode, QPushButton] = {}
        mode_defs = [
            (TelemetryMode.MOCK, "Mock"),
            (TelemetryMode.ROS2_NATIVE, "ROS 2 (rclpy)"),
            (TelemetryMode.ROSBRIDGE, "Rosbridge (WS)"),
        ]
        for mode, label in mode_defs:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setChecked(mode == TelemetryMode.MOCK)
            btn.setStyleSheet(
                "QPushButton { background: #000000; color: #94a3b8; border: 1px solid #363636; border-radius: 6px; padding: 5px 10px; font-size: 11px; }"
                "QPushButton:checked { background: #0e7490; color: #ffffff; border-color: #22d3ee; }"
            )
            btn.clicked.connect(lambda _chk, m=mode: self._on_mode_clicked(m))
            if mode == TelemetryMode.ROS2_NATIVE and not is_rclpy_available():
                btn.setToolTip(
                    "rclpy não carregado neste ambiente Python. Inicie após `source /opt/ros/humble/setup.bash`."
                )
            mode_group.addButton(btn)
            self._mode_buttons[mode] = btn
            layout.addWidget(btn)

        # Campos IP / Porta para Rosbridge
        cfg = self._store.get_config()
        ws_label = QLabel("ws://")
        ws_label.setStyleSheet("color: #64748b; font-family: monospace; font-size: 11px;")
        layout.addWidget(ws_label)

        self._host_input = QLineEdit(cfg.host)
        self._host_input.setFixedWidth(115)
        self._host_input.setStyleSheet(
            "background: #000000; color: #f8fafc; border: 1px solid #363636; border-radius: 5px; padding: 4px 6px; font-family: monospace; font-size: 11px;"
        )
        layout.addWidget(self._host_input)

        self._port_input = QLineEdit(str(cfg.port))
        self._port_input.setFixedWidth(54)
        self._port_input.setStyleSheet(
            "background: #000000; color: #f8fafc; border: 1px solid #363636; border-radius: 5px; padding: 4px 6px; font-family: monospace; font-size: 11px;"
        )
        layout.addWidget(self._port_input)

        connect_btn = QPushButton("Conectar")
        connect_btn.clicked.connect(self._on_apply_connection)
        connect_btn.setStyleSheet(
            "QPushButton { background: #242424; color: #e2e8f0; border: 1px solid #363636; border-radius: 6px; padding: 5px 10px; font-size: 11px; }"
            "QPushButton:hover { background: #363636; }"
        )
        layout.addWidget(connect_btn)

        self._topics_btn = QPushButton("Tópicos")
        self._topics_btn.setCheckable(True)
        self._topics_btn.clicked.connect(
            lambda checked: self._topics_drawer.setVisible(checked)
        )
        self._topics_btn.setStyleSheet(
            "QPushButton { background: #000000; color: #cbd5e1; border: 1px solid #363636; border-radius: 6px; padding: 5px 10px; font-size: 11px; }"
            "QPushButton:checked { background: #141414; color: #67e8f9; border-color: #06b6d4; }"
        )
        layout.addWidget(self._topics_btn)

        self._pause_btn = QPushButton("Pausar")
        self._pause_btn.clicked.connect(self._on_toggle_pause)
        self._pause_btn.setStyleSheet(
            "QPushButton { background: #242424; color: #f8fafc; border: 1px solid #363636; border-radius: 6px; padding: 5px 12px; font-size: 11px; font-weight: bold; }"
            "QPushButton:hover { background: #363636; }"
        )
        layout.addWidget(self._pause_btn)

        self._csv_btn = QPushButton("Exportar CSV (0)")
        self._csv_btn.clicked.connect(self._on_export_csv)
        self._csv_btn.setStyleSheet(
            "QPushButton { background: #141414; color: #67e8f9; border: 1px solid #06b6d4; border-radius: 6px; padding: 5px 12px; font-size: 11px; font-weight: bold; }"
            "QPushButton:hover { background: #0e7490; color: #ffffff; }"
        )
        layout.addWidget(self._csv_btn)

        reset_btn = QPushButton("Limpar")
        reset_btn.clicked.connect(self._on_reset_data)
        reset_btn.setStyleSheet(
            "QPushButton { background: #000000; color: #94a3b8; border: 1px solid #363636; border-radius: 6px; padding: 5px 9px; font-size: 11px; }"
            "QPushButton:hover { color: #f8fafc; }"
        )
        layout.addWidget(reset_btn)

        return bar

    def _build_topics_drawer(self) -> QFrame:
        drawer = QFrame()
        drawer.setVisible(False)
        drawer.setStyleSheet("background-color: #000000; border-bottom: 1px solid #242424;")
        layout = QHBoxLayout(drawer)
        layout.setContentsMargins(12, 6, 12, 6)
        layout.setSpacing(10)

        cfg = self._store.get_config().topics
        input_style = (
            "background: #0d0d0d; color: #f8fafc; border: 1px solid #363636; "
            "border-radius: 5px; padding: 3px 8px; font-family: monospace; font-size: 11px;"
        )

        for label_text, attr_name, initial in (
            ("cmd_vel:", "_cmd_topic_input", cfg.cmd_vel),
            ("odom:", "_odom_topic_input", cfg.odom),
            ("PID Debug:", "_pid_topic_input", cfg.pid),
            ("PID Gains:", "_pid_gains_topic_input", cfg.pid_gains),
        ):
            lbl = QLabel(label_text)
            lbl.setStyleSheet("color: #94a3b8; font-size: 11px;")
            edit = QLineEdit(initial)
            edit.setStyleSheet(input_style)
            setattr(self, attr_name, edit)
            layout.addWidget(lbl)
            layout.addWidget(edit)

        apply_btn = QPushButton("Aplicar Tópicos")
        apply_btn.clicked.connect(self._on_apply_connection)
        apply_btn.setStyleSheet(
            "QPushButton { background: #0e7490; color: #ffffff; border: none; border-radius: 5px; padding: 4px 12px; font-size: 11px; }"
        )
        layout.addWidget(apply_btn)
        layout.addStretch()

        self._topics_drawer = drawer
        return drawer

    def _build_rates_and_mock_bar(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet("background-color: #090909; border-bottom: 1px solid #242424;")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(12, 6, 12, 6)
        layout.setSpacing(12)

        rates_lbl = QLabel("Taxas:")
        rates_lbl.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: bold;")
        layout.addWidget(rates_lbl)

        self._rate_cmd_lbl = QLabel("/cmd_vel: 0.0 Hz")
        self._rate_odom_lbl = QLabel("/odom: 0.0 Hz")
        self._rate_pid_lbl = QLabel("/tigo/pid_debug: 0.0 Hz")
        for lbl in (self._rate_cmd_lbl, self._rate_odom_lbl, self._rate_pid_lbl):
            lbl.setStyleSheet(
                "background: #000000; color: #94a3b8; border: 1px solid #242424; border-radius: 5px; padding: 3px 8px; font-family: monospace; font-size: 11px;"
            )
            layout.addWidget(lbl)

        self._paused_banner = QLabel("PAUSADO PARA INSPEÇÃO")
        self._paused_banner.setVisible(False)
        self._paused_banner.setStyleSheet(
            "background: #451a03; color: #fcd34d; border-radius: 4px; padding: 3px 8px; font-size: 11px; font-weight: bold;"
        )
        layout.addWidget(self._paused_banner)

        layout.addStretch()

        # Controles específicos do simulador Mock
        self._mock_controls = QWidget()
        mock_layout = QHBoxLayout(self._mock_controls)
        mock_layout.setContentsMargins(0, 0, 0, 0)
        mock_layout.setSpacing(8)

        sim_lbl = QLabel("Simulador:")
        sim_lbl.setStyleSheet("color: #67e8f9; font-size: 11px; font-weight: bold;")
        mock_layout.addWidget(sim_lbl)

        self._scenario_combo = QComboBox()
        for scenario, text in MOCK_SCENARIO_LABELS.items():
            self._scenario_combo.addItem(text, scenario)
        self._scenario_combo.currentIndexChanged.connect(self._on_scenario_changed)
        self._scenario_combo.setStyleSheet(
            "background: #000000; color: #e2e8f0; border: 1px solid #363636; border-radius: 5px; padding: 3px 8px; font-size: 11px;"
        )
        mock_layout.addWidget(self._scenario_combo)

        self._rate_slider = QSlider(Qt.Horizontal)
        self._rate_slider.setRange(20, 100)
        self._rate_slider.setSingleStep(5)
        self._rate_slider.setValue(50)
        self._rate_slider.setFixedWidth(90)
        self._rate_slider.valueChanged.connect(self._on_mock_rate_changed)
        mock_layout.addWidget(self._rate_slider)

        self._mock_hz_lbl = QLabel("50 Hz")
        self._mock_hz_lbl.setStyleSheet("color: #67e8f9; font-family: monospace; font-size: 11px;")
        mock_layout.addWidget(self._mock_hz_lbl)

        layout.addWidget(self._mock_controls)
        return bar

    def _on_mode_clicked(self, mode: TelemetryMode) -> None:
        self._teleop_panel.stop_robot()
        self._store.set_mode(mode)
        self._mock_controls.setVisible(mode == TelemetryMode.MOCK)
        self._on_ui_tick()

    def _on_teleop_triggered(self) -> None:
        if self._store.get_snapshot().mode == TelemetryMode.MOCK:
            idx = self._scenario_combo.findData(MockScenario.MANUAL)
            if idx >= 0 and self._scenario_combo.currentIndex() != idx:
                self._scenario_combo.blockSignals(True)
                self._scenario_combo.setCurrentIndex(idx)
                self._scenario_combo.blockSignals(False)

    def _on_apply_connection(self) -> None:
        try:
            port = int(self._port_input.text().strip())
        except ValueError:
            port = 9090
        cfg = ConnectionConfig(
            host=self._host_input.text().strip() or "localhost",
            port=port if port > 0 else 9090,
            topics=TopicNamesConfig(
                cmd_vel=self._cmd_topic_input.text().strip() or "/cmd_vel",
                odom=self._odom_topic_input.text().strip() or "/odom",
                pid=self._pid_topic_input.text().strip() or "/tigo/pid_debug",
                pid_gains=self._pid_gains_topic_input.text().strip() or "/tigo/pid_gains",
            ),
        )
        snap = self._store.get_snapshot()
        if snap.mode == TelemetryMode.MOCK:
            self._store.update_connection_config(cfg, reconnect=False)
            self._mode_buttons[TelemetryMode.ROSBRIDGE].setChecked(True)
            self._store.set_mode(TelemetryMode.ROSBRIDGE)
            self._mock_controls.setVisible(False)
        else:
            self._store.update_connection_config(cfg, reconnect=True)

    def _on_scenario_changed(self) -> None:
        scenario = self._scenario_combo.currentData()
        if isinstance(scenario, MockScenario):
            self._store.set_mock_scenario(scenario)

    def _on_mock_rate_changed(self, value: int) -> None:
        self._mock_hz_lbl.setText(f"{value} Hz")
        self._store.set_mock_rate_hz(value)

    def _on_toggle_pause(self) -> None:
        paused = self._store.toggle_paused()
        self._pause_btn.setText("Retomar" if paused else "Pausar")
        self._paused_banner.setVisible(paused)

    def _on_reset_data(self) -> None:
        self._store.reset_data()
        self._on_chart_tick()
        self._on_ui_tick()

    def _on_export_csv(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"tigo_telemetry_{stamp}.csv"
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Exportar Telemetria para CSV",
            default_name,
            "Arquivos CSV (*.csv);;Todos os arquivos (*)",
        )
        if not path:
            return
        count = self._store.export_csv(path)
        QMessageBox.information(
            self,
            "Exportação Concluída",
            f"{count} amostras exportadas para:\n{path}",
        )

    def _on_chart_tick(self) -> None:
        snap = self._store.get_snapshot()
        self._odom_panel.refresh_map()
        if snap.paused:
            return
        self._cmd_panel.refresh_charts(snap.cmd_vel)
        self._pid_panel.refresh_charts()

    def _on_ui_tick(self) -> None:
        snap: TelemetrySnapshot = self._store.get_snapshot()
        text, css = _STATUS_STYLES[snap.connection_state]
        self._status_badge.setText(text)
        self._status_badge.setToolTip(snap.status_detail)
        self._status_badge.setStyleSheet(
            f"background-color: {css}; border-radius: 10px; padding: 3px 10px; font-size: 11px; font-weight: bold;"
        )

        self._csv_btn.setText(f"Exportar CSV ({snap.sample_count})")
        self._csv_btn.setEnabled(snap.sample_count > 0)

        topics = self._store.get_config().topics
        self._update_rate_label(self._rate_cmd_lbl, topics.cmd_vel, snap.rates.cmd_vel_hz)
        self._update_rate_label(self._rate_odom_lbl, topics.odom, snap.rates.odom_hz)
        self._update_rate_label(self._rate_pid_lbl, topics.pid, snap.rates.pid_hz)

        self._cmd_panel.refresh_numbers(snap.cmd_vel)
        self._odom_panel.refresh_numbers(snap.odom)
        self._pid_panel.refresh_numbers(snap.pid)

    @staticmethod
    def _update_rate_label(label: QLabel, topic: str, hz: float) -> None:
        label.setText(f"{topic}: {hz:.1f} Hz")
        color = "#6ee7b7" if hz > 0.2 else "#64748b"
        label.setStyleSheet(
            f"background: #000000; color: {color}; border: 1px solid #242424; border-radius: 5px; padding: 3px 8px; font-family: monospace; font-size: 11px;"
        )

    def closeEvent(self, event: object) -> None:
        self._teleop_panel.stop_robot()
        self._teleop_timer.stop()
        self._chart_timer.stop()
        self._ui_timer.stop()
        self._store.stop()
        super().closeEvent(event)
