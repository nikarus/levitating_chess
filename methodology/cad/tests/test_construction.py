import unittest
from unittest.mock import patch

import numpy as np

from model import Inputs, Fixed
from methodology.cad import winding


class WindingConstructionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path, cls.turns, cls.reference = winding.continuous_path()
        cls.clearance = winding.path_clearance(cls.path)

    def test_continuous_turns_keep_circulation_direction_across_layer_changes(self):
        self.assertEqual(len(self.turns), self.reference.turns)
        for turn in self.turns:
            vertices = np.array([tuple(edge @ 0)[:2] for edge in turn] + [tuple(turn[-1] @ 1)[:2]])
            cross = vertices[:, 0] * np.roll(vertices[:, 1], -1) - vertices[:, 1] * np.roll(vertices[:, 0], -1)
            self.assertGreater(cross.sum(), 0)
        edges = self.path.edges().sort_by(self.path)
        for first, second in zip(edges, edges[1:]):
            self.assertLess((first @ 1 - second @ 0).length, Fixed.cad_geometry_tolerance)
            self.assertAlmostEqual((first % 1).dot(second % 0), 1)
        self.assertAlmostEqual((self.path @ 0).Z, (self.path @ 1).Z)
        self.assertAlmostEqual(abs((self.path @ 0).X - (self.path @ 1).X), Fixed.prototype_lead_separation)

    def test_layer_detours_are_required_to_avoid_enamel_overlap(self):
        self.assertGreater(self.clearance['minimum_insulated_clearance_mm'], Fixed.cad_geometry_tolerance)
        with patch.object(Fixed, 'winding_layer_transition_bulge', 0):
            direct, _, _ = winding.continuous_path()
        self.assertLess(winding.path_clearance(direct)['minimum_insulated_clearance_mm'], -Fixed.cad_geometry_tolerance)

    def test_wire_height_and_terminal_strip_fit_existing_stack(self):
        points = np.array([tuple(edge.position_at(fraction)) for edge in self.path.edges() for fraction in (0, 0.5, 1)])
        self.assertGreaterEqual(points[:, 2].min() - Fixed.prototype_wire_outer_diameter / 2, 0)
        self.assertLessEqual(points[:, 2].max() + Fixed.prototype_wire_outer_diameter / 2, self.reference.coil_height)
        pad_width, pad_length, _ = Fixed.winding_terminal_pad_size
        self.assertLess(pad_width, Fixed.prototype_lead_separation)
        self.assertLess(pad_length, self.path.edges().sort_by(self.path)[0].length)
        self.assertLess(pad_length, self.path.edges().sort_by(self.path)[-1].length + Fixed.cad_geometry_tolerance)


if __name__ == '__main__':
    unittest.main()
