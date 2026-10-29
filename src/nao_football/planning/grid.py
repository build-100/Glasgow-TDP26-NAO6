"""Nine regions are planning targets, not exact robot or ball positions."""

from dataclasses import dataclass
from math import hypot, isfinite
from typing import Optional, Tuple


@dataclass(frozen=True)
class FieldGrid:
    length_m: float = 9.0
    width_m: float = 6.0

    def __post_init__(self):
        if not all(isfinite(v) and v > 0 for v in (self.length_m, self.width_m)):
            raise ValueError("Field dimensions must be finite and positive.")

    @staticmethod
    def _check(node: int):
        if not isinstance(node, int) or isinstance(node, bool) or not 0 <= node < 9:
            raise ValueError("Node must be an integer in 0..8.")

    def neighbours(self, node: int) -> Tuple[int, ...]:
        self._check(node)
        row, col = divmod(node, 3)
        return tuple(n for n in range(9) if n != node
                     and abs(n // 3 - row) <= 1 and abs(n % 3 - col) <= 1)

    def legal_moves(self, node: int) -> Tuple[int, ...]:
        return tuple(sorted((node,) + self.neighbours(node)))

    def edges(self) -> Tuple[Tuple[int, int], ...]:
        return tuple((u, v) for u in range(9) for v in self.neighbours(u) if u < v)

    def centre(self, node: int) -> Tuple[float, float]:
        self._check(node)
        row, col = divmod(node, 3)
        return ((col + 0.5) * self.length_m / 3,
                (row + 0.5) * self.width_m / 3)

    def region(self, x: float, y: float) -> Optional[int]:
        # Half-open cells give reproducible boundary ownership. Outer upper
        # boundaries are out of field; goal crossing needs its own event rule.
        if not (isfinite(x) and isfinite(y) and
                0 <= x < self.length_m and 0 <= y < self.width_m):
            return None
        return min(2, int(y / (self.width_m / 3))) * 3 + min(2, int(x / (self.length_m / 3)))

    def move_distance(self, source: int, target: int) -> float:
        if target not in self.legal_moves(source):
            raise ValueError("Movement must stay or follow one adjacent edge.")
        a, b = self.centre(source), self.centre(target)
        return hypot(b[0] - a[0], b[1] - a[1])
