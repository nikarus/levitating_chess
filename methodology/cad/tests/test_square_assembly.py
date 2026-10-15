import unittest

import numpy as np

from model import Inputs, Fixed
from methodology import prototype_coils
from methodology.cad import square_assembly


class SquareAssemblyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parts, cls.array, cls.winding, cls.piece = square_assembly.square_parts()

    def test_physical_patch_preserves_force_model_positions_and_single_layer(self):
        coils = [part.shape for part in self.parts if part.group == 'Single_layer_coils']
        self.assertEqual(len(coils), Fixed.cad_patch_cells_per_side ** 2)
        reference = prototype_coils.prototype_array(Inputs.cad_coil_layout)
        for identity, filaments in zip(reference.identities, reference.coils):
            index = self.array.identities.index(identity)
            np.testing.assert_array_equal(filaments[0].vertices, self.array.coils[index][0].vertices)
        for shape, placement in zip(coils, self.array.placements):
            box = shape.bounding_box()
            expected_center = self.array.rotation.apply([placement['x'], placement['y'], 0]) * 1000
            np.testing.assert_allclose([box.center().X, box.center().Y], expected_center[:2], atol=Fixed.cad_geometry_tolerance)
            self.assertAlmostEqual(box.max.Z, -Inputs.magnet_to_coil_distance)
            self.assertAlmostEqual(box.size.Z, self.winding.coil_height)
        self.assertAlmostEqual(coils[0].distance_to(coils[1]), Inputs.prototype_coil_spacing)

    def test_conductive_stack_matches_motion_screen_depths(self):
        by_group = {part.group: part.shape for part in self.parts}
        pcb = by_group['PCB'].bounding_box()
        plate = by_group['Baseplate'].bounding_box()
        self.assertAlmostEqual(-pcb.center().Z,
            Inputs.magnet_to_coil_distance + self.winding.coil_height + Fixed.pcb_thickness / 2)
        self.assertAlmostEqual(-plate.max.Z,
            Inputs.magnet_to_coil_distance + self.winding.coil_height + Fixed.pcb_thickness + Fixed.radiator_standoff_below_pcb)
        surface = by_group['Playing_surface'].bounding_box()
        skin = by_group['Piece_skin'].bounding_box()
        self.assertAlmostEqual(skin.min.Z - surface.max.Z, Inputs.magnet_to_coil_distance -
            Fixed.potting_cover_thickness - Fixed.playing_surface_thickness - Fixed.piece_bottom_skin)

    def test_king_outline_contains_existing_magnets_and_matches_dimensions(self):
        king = next(part.shape for part in self.parts if part.group == 'King_envelope')
        self.assertTrue(king.is_valid)
        box = king.bounding_box()
        self.assertAlmostEqual(box.size.Z, self.piece.box_height)
        self.assertAlmostEqual(box.size.X, self.piece.diameter)
        magnets = [part.shape for part in self.parts if part.group == 'Magnets']
        for magnet in magnets:
            self.assertLess((magnet - king).volume, Fixed.cad_geometry_tolerance ** 3)
        self.assertAlmostEqual(min(magnet.bounding_box().min.Z for magnet in magnets), 0)


if __name__ == '__main__':
    unittest.main()
