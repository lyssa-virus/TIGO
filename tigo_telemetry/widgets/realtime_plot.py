import bisect
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QMouseEvent, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import QSizePolicy, QWidget


@dataclass(frozen=True)
class PlotSeriesSpec:
    label: str
    color: str
    dashed: bool = False
    right_axis: bool = False
    unit: str = ""


class RealtimePlotWidget(QWidget):
    """
    Gráfico temporal leve em QPainter otimizado para 25–60 FPS.
    Suporta eixo Y duplo e crosshair interativo para inspeção de picos quando pausado.
    """

    def __init__(
        self,
        series_specs: Sequence[PlotSeriesSpec],
        right_range: Optional[Tuple[float, float]] = None,
        min_height: int = 200,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._specs = list(series_specs)
        self._right_range = right_range
        self._times: List[float] = []
        self._values: List[List[float]] = [[] for _ in self._specs]
        self._hover_x: Optional[int] = None

        self.setMinimumHeight(min_height)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

    def set_data(self, times: List[float], values: List[List[float]]) -> None:
        self._times = times
        self._values = values
        self.update()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        self._hover_x = event.x()
        self.update()

    def leaveEvent(self, _event: object) -> None:
        self._hover_x = None
        self.update()

    @staticmethod
    def _compute_bounds(
        series_list: List[List[float]],
        indices: List[int],
        fixed: Optional[Tuple[float, float]] = None,
    ) -> Tuple[float, float]:
        if fixed is not None:
            return fixed
        lo = float("inf")
        hi = float("-inf")
        for idx in indices:
            if idx >= len(series_list):
                continue
            for v in series_list[idx]:
                if v < lo:
                    lo = v
                if v > hi:
                    hi = v
        if lo == float("inf") or hi == float("-inf"):
            return -1.0, 1.0
        span = hi - lo
        if span < 0.05:
            mid = 0.5 * (lo + hi)
            return mid - 0.5, mid + 0.5
        pad = span * 0.12
        return lo - pad, hi + pad

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        w = self.width()
        h = self.height()
        painter.fillRect(0, 0, w, h, QColor("#000000"))

        has_right = any(s.right_axis for s in self._specs)
        margin_left = 54
        margin_right = 52 if has_right else 14
        margin_top = 28
        margin_bottom = 26

        plot_w = max(40, w - margin_left - margin_right)
        plot_h = max(40, h - margin_top - margin_bottom)
        plot_rect = QRectF(margin_left, margin_top, plot_w, plot_h)

        # Borda da área do gráfico
        painter.setPen(QPen(QColor("#242424"), 1))
        painter.drawRect(plot_rect)

        left_indices = [i for i, s in enumerate(self._specs) if not s.right_axis]
        right_indices = [i for i, s in enumerate(self._specs) if s.right_axis]

        y_left_min, y_left_max = self._compute_bounds(self._values, left_indices)
        y_right_min, y_right_max = self._compute_bounds(
            self._values, right_indices, fixed=self._right_range
        )

        font = QFont("Monospace", 8)
        painter.setFont(font)

        # Grade horizontal e ticks dos eixos Y
        steps = 4
        for step in range(steps + 1):
            frac = step / steps
            yy = plot_rect.bottom() - frac * plot_h
            painter.setPen(QPen(QColor(50, 50, 50, 125), 1))
            painter.drawLine(QPointF(plot_rect.left(), yy), QPointF(plot_rect.right(), yy))

            val_left = y_left_min + frac * (y_left_max - y_left_min)
            painter.setPen(QColor("#94a3b8"))
            painter.drawText(
                QRectF(2, yy - 8, margin_left - 6, 16),
                Qt.AlignRight | Qt.AlignVCenter,
                f"{val_left:.2f}",
            )

            if has_right:
                val_right = y_right_min + frac * (y_right_max - y_right_min)
                painter.setPen(QColor("#fbbf24"))
                painter.drawText(
                    QRectF(plot_rect.right() + 4, yy - 8, margin_right - 6, 16),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    f"{val_right:.0f}",
                )

        n = len(self._times)
        if n < 2:
            painter.setPen(QColor("#64748b"))
            painter.drawText(plot_rect, Qt.AlignCenter, "Aguardando amostras...")
            self._draw_legend(painter, w, None)
            return

        t_min = self._times[0]
        t_max = self._times[-1]
        t_span = max(0.001, t_max - t_min)

        # Grade vertical e ticks do eixo X
        x_steps = 5
        for step in range(x_steps + 1):
            frac = step / x_steps
            xx = plot_rect.left() + frac * plot_w
            painter.setPen(QPen(QColor(50, 50, 50, 125), 1))
            painter.drawLine(QPointF(xx, plot_rect.top()), QPointF(xx, plot_rect.bottom()))
            t_val = t_min + frac * t_span
            painter.setPen(QColor("#94a3b8"))
            painter.drawText(
                QRectF(xx - 24, plot_rect.bottom() + 3, 48, 18),
                Qt.AlignCenter,
                f"{t_val:.1f}s",
            )

        # Clipping para desenhar apenas dentro da área do gráfico
        painter.save()
        painter.setClipRect(plot_rect)

        for idx, spec in enumerate(self._specs):
            vals = self._values[idx] if idx < len(self._values) else []
            if len(vals) < 2:
                continue
            y_min, y_max = (
                (y_right_min, y_right_max) if spec.right_axis else (y_left_min, y_left_max)
            )
            y_span = max(1e-6, y_max - y_min)

            path = QPainterPath()
            for j in range(min(n, len(vals))):
                px = plot_rect.left() + ((self._times[j] - t_min) / t_span) * plot_w
                py = plot_rect.bottom() - ((vals[j] - y_min) / y_span) * plot_h
                if j == 0:
                    path.moveTo(px, py)
                else:
                    path.lineTo(px, py)

            pen = QPen(QColor(spec.color), 1.8)
            if spec.dashed:
                pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.drawPath(path)

        # Crosshair de inspeção ao passar o mouse
        hover_idx: Optional[int] = None
        if self._hover_x is not None and plot_rect.left() <= self._hover_x <= plot_rect.right():
            target_t = t_min + ((self._hover_x - plot_rect.left()) / plot_w) * t_span
            pos = bisect.bisect_left(self._times, target_t)
            if pos >= n:
                hover_idx = n - 1
            elif pos == 0:
                hover_idx = 0
            else:
                before = self._times[pos - 1]
                after = self._times[pos]
                hover_idx = pos - 1 if (target_t - before) <= (after - target_t) else pos

            hx = plot_rect.left() + ((self._times[hover_idx] - t_min) / t_span) * plot_w
            cross_pen = QPen(QColor("#cbd5e1"), 1, Qt.DashLine)
            painter.setPen(cross_pen)
            painter.drawLine(QPointF(hx, plot_rect.top()), QPointF(hx, plot_rect.bottom()))

        painter.restore()
        self._draw_legend(painter, w, hover_idx)

    def _draw_legend(
        self, painter: QPainter, width: int, hover_idx: Optional[int]
    ) -> None:
        x_cursor = 10.0
        y_top = 6.0
        painter.setFont(QFont("Monospace", 8))

        if hover_idx is not None and hover_idx < len(self._times):
            t_label = f"t={self._times[hover_idx]:.3f}s | "
            painter.setPen(QColor("#e2e8f0"))
            painter.drawText(QPointF(x_cursor, y_top + 11), t_label)
            x_cursor += painter.fontMetrics().horizontalAdvance(t_label) + 4

        for idx, spec in enumerate(self._specs):
            if x_cursor > width - 80:
                break
            vals = self._values[idx] if idx < len(self._values) else []
            sample_idx = (
                hover_idx
                if (hover_idx is not None and hover_idx < len(vals))
                else (len(vals) - 1)
            )
            val_str = f"{vals[sample_idx]:.2f}" if sample_idx >= 0 else "--"
            text = f"{spec.label}: {val_str}{(' ' + spec.unit) if spec.unit else ''}"

            painter.fillRect(QRectF(x_cursor, y_top + 3, 10, 8), QColor(spec.color))
            x_cursor += 14
            painter.setPen(QColor("#cbd5e1"))
            painter.drawText(QPointF(x_cursor, y_top + 11), text)
            x_cursor += painter.fontMetrics().horizontalAdvance(text) + 14
