from collections import deque
import time
from typing import Deque, Generic, List, Optional, TypeVar

T = TypeVar("T")


class RingBuffer(Generic[T]):
    """Buffer circular de capacidade fixa baseado em deque para ingestão em alta frequência."""

    def __init__(self, capacity: int) -> None:
        self._buf: Deque[T] = deque(maxlen=max(1, capacity))

    def push(self, item: T) -> None:
        self._buf.append(item)

    def clear(self) -> None:
        self._buf.clear()

    def latest(self) -> Optional[T]:
        return self._buf[-1] if self._buf else None

    def to_list(self) -> List[T]:
        return list(self._buf)

    def __len__(self) -> int:
        return len(self._buf)


class RateTracker:
    """Mede a frequência real de mensagens (Hz) usando janela deslizante."""

    def __init__(self, window_sec: float = 1.5) -> None:
        self._window_sec = window_sec
        self._stamps: Deque[float] = deque()

    def tick(self, now_sec: Optional[float] = None) -> None:
        t = time.monotonic() if now_sec is None else now_sec
        self._stamps.append(t)
        self._prune(t)

    def get_hz(self, now_sec: Optional[float] = None) -> float:
        t = time.monotonic() if now_sec is None else now_sec
        self._prune(t)
        n = len(self._stamps)
        if n < 2:
            return 0.0
        elapsed = t - self._stamps[0]
        if elapsed <= 0.05:
            return 0.0
        return round((n - 1) / elapsed, 1)

    def reset(self) -> None:
        self._stamps.clear()

    def _prune(self, now_sec: float) -> None:
        cutoff = now_sec - self._window_sec
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()
