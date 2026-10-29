import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from nao_football.planning.grid import FieldGrid
from nao_football.policy_stub import select_movement_targets


class FoundationTests(unittest.TestCase):
    def test_no_long_edges_or_boundary_wrap(self):
        grid = FieldGrid()
        self.assertEqual(len(grid.edges()), 20)
        self.assertEqual(set(grid.neighbours(0)), {1, 3, 4})
        for a, b in grid.edges():
            self.assertIn(a, grid.neighbours(b))
        with self.assertRaises(ValueError):
            grid.move_distance(0, 8)

    def test_region_centres_and_outside_field(self):
        grid = FieldGrid()
        for n in range(9):
            self.assertEqual(grid.region(*grid.centre(n)), n)
        self.assertIsNone(grid.region(9, 1))

    def test_placeholder_does_not_claim_movement(self):
        self.assertEqual(select_movement_targets([0, 3, 6]), (0, 3, 6))
        with self.assertRaises(ValueError):
            select_movement_targets([0, 9, 6])

    def test_model_retains_physical_descriptions(self):
        spec = importlib.util.spec_from_file_location("check_model", ROOT / "scripts/check_model.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        result = module.check()
        self.assertEqual(result["simplified"]["independent_moving_joints"], 21)
        self.assertEqual(result["simplified"]["collisions"], result["full"]["collisions"])


if __name__ == "__main__":
    unittest.main()
