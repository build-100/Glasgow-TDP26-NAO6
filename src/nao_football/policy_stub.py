"""Interface scaffold only: no private strategy or learned policy is included."""
from typing import Sequence, Tuple


def select_movement_targets(player_nodes: Sequence[int]) -> Tuple[int, ...]:
    """Safe initial placeholder: each field player waits in its current region."""
    if len(player_nodes) != 3 or any(not isinstance(n, int) or isinstance(n, bool)
                                     or not 0 <= n < 9 for n in player_nodes):
        raise ValueError("Provide three field-player node IDs in 0..8.")
    return tuple(player_nodes)
