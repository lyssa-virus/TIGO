from typing import Optional

from PyQt5.QtWidgets import QFrame, QLabel, QVBoxLayout, QWidget


class MetricCard(QFrame):
    """Cartão compacto reutilizável para exibição numérica em tempo real."""

    def __init__(
        self,
        title: str,
        subtitle: str = "",
        color: str = "#22d3ee",
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("metricCard")
        self.setStyleSheet(
            "#metricCard { background-color: #000000; border: 1px solid #242424; border-radius: 8px; }"
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 7, 10, 7)
        layout.setSpacing(2)

        self._title = QLabel(title)
        self._title.setStyleSheet("color: #94a3b8; font-size: 11px;")
        layout.addWidget(self._title)

        self._value = QLabel("--")
        self._value.setStyleSheet(
            f"color: {color}; font-family: monospace; font-size: 16px; font-weight: bold;"
        )
        layout.addWidget(self._value)

        if subtitle:
            self._sub = QLabel(subtitle)
            self._sub.setStyleSheet("color: #64748b; font-size: 10px;")
            layout.addWidget(self._sub)

    def set_value(self, text: str) -> None:
        self._value.setText(text)
