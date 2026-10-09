import unittest
from unittest.mock import patch

import build123d as cad

from model import Inputs, Fixed
from methodology.cad import assembly


class AssemblyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parts, cls.board, cls.coil, cls.winding, cls.piece = assembly.prototype_parts()

    def test_gap_change_moves_board_and_preserves_magnet_origin(self):
        gap_change = Inputs.magnet_to_coil_distance
        with patch.object(Inputs, 'magnet_to_coil_distance', Inputs.magnet_to_coil_distance + gap_change):
            shifted, *_ = assembly.prototype_parts()
        for original, moved in zip(self.parts, shifted):
            delta = moved.shape.bounding_box().min.Z - original.shape.bounding_box().min.Z
            expected = 0 if original.group in ('Magnets', 'Piece_reference') else -gap_change
            self.assertAlmostEqual(delta, expected)

    def test_two_coil_layers_have_modeled_gap_height_and_channel_count(self):
        upper = [part.shape for part in self.parts if part.group == 'Upper_coils']
        lower = [part.shape for part in self.parts if part.group == 'Lower_coils']
        self.assertEqual(len(upper) + len(lower), self.coil.control_bed_bodies)
        self.assertAlmostEqual(max(shape.bounding_box().max.Z for shape in upper), -Inputs.magnet_to_coil_distance)
        self.assertAlmostEqual(min(shape.bounding_box().min.Z for shape in upper), max(shape.bounding_box().max.Z for shape in lower))
        self.assertAlmostEqual(min(shape.bounding_box().min.Z for shape in lower),
                               -Inputs.magnet_to_coil_distance - Fixed.herringbone_orientation_families * self.winding.coil_height)

    def test_copper_and_insulation_packing_match_electrical_geometry(self):
        parts = assembly.winding_detail(self.coil, self.winding)
        copper = [part.shape for part in parts if part.group == 'Copper']
        insulation = [part.shape for part in parts if part.group == 'Insulation']
        self.assertEqual(len(copper), self.winding.turns)
        self.assertAlmostEqual(sum(shape.volume for shape in copper),
                               self.winding.length_per_winding * self.winding.cross_section_area * 1000000000)
        self.assertAlmostEqual(max(shape.bounding_box().max.Z for shape in insulation), self.winding.coil_height)
        self.assertAlmostEqual(copper[0].bounding_box().min.Z, Fixed.rectangular_wire_film)
        assembly.validate_parts(parts)

    def test_collision_checker_rejects_overlapping_solids(self):
        first = assembly.labeled_part(cad.Box(1, 1, 1), 'first', 'red', 'Test', 0)
        second = assembly.labeled_part(cad.Box(1, 1, 1), 'second', 'blue', 'Test', 0)
        with self.assertRaisesRegex(ValueError, 'Overlapping CAD material regions'):
            assembly.validate_parts([first, second])


if __name__ == '__main__':
    unittest.main()
