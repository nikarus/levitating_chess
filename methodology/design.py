import argparse
from contextlib import redirect_stdout
from fractions import Fraction
from io import StringIO
from math import pi, sqrt, exp, expm1, ceil, floor, log, radians, log10, log2, isfinite, lcm
from pathlib import Path

import levitation_sim


from model import Inputs, Fixed, Constants


def interpolate_height_coupling(sim, coil_height_mm):
    heights = sim["coil_height_coupling_heights_mm"]
    factors = sim["coil_height_coupling_factors"]
    if not heights[0] <= coil_height_mm <= heights[-1]:
        raise ValueError("Coil height outside measured coupling range")
    if coil_height_mm == heights[0]:
        return factors[0]
    for index in range(1, len(heights)):
        if coil_height_mm <= heights[index]:
            span = heights[index] - heights[index - 1]
            weight = (coil_height_mm - heights[index - 1]) / span
            return factors[index - 1] + weight * (factors[index] - factors[index - 1])


class Cell:
    def __init__(self, label, value, unit=""):
        self.label = label
        self.value = value
        self.unit = unit


class Wire:
    def __init__(self, label, radial_pitch, axial_pitch, copper_area):
        self.label = label
        self.radial_pitch = radial_pitch
        self.axial_pitch = axial_pitch
        self.copper_area = copper_area


def rectangular_wire(width, thickness):
    film = 2 * Fixed.rectangular_wire_film
    return Wire(f"{width:g}x{thickness:g}mm flat", width + film, thickness + film, width * thickness / 1000000)


class BoardGeometry:
    def __init__(self):
        self.period_length = Inputs.magnets_per_period * Inputs.magnet_lateral_edge
        self.platform_side = Inputs.periods_per_side * self.period_length
        self.base_diameter = self.platform_side * sqrt(2) + 2 * Fixed.base_corner_standoff
        self.min_square_size = self.base_diameter / Fixed.square_fill_ratio
        self.periods_per_square = ceil(self.min_square_size / self.period_length)
        self.square_size = self.periods_per_square * self.period_length
        self.square_fill = self.base_diameter / self.square_size
        self.board_side = Constants.board_squares_per_side * self.square_size
        self.captured_per_side = ceil(Inputs.piece_count / Fixed.captured_side_areas)
        self.storage_rows = Constants.board_squares_per_side
        self.storage_columns = ceil(self.captured_per_side / self.storage_rows)
        self.storage_width_each = self.storage_columns * self.square_size
        self.motor_width = self.board_side + Fixed.captured_side_areas * self.storage_width_each
        self.motor_height = self.board_side
        self.motor_area = self.motor_width * self.motor_height

    def cells(self):
        return [
            Cell("Magnetic period length", self.period_length, "mm"),
            Cell("Platform square side", self.platform_side, "mm"),
            Cell("Round base diameter", self.base_diameter, "mm"),
            Cell("Minimum square size (fill ratio)", self.min_square_size, "mm"),
            Cell("Chess square size (snapped to periods)", self.square_size, "mm"),
            Cell("Magnet periods per square", self.periods_per_square),
            Cell("Square fill (base / square)", self.square_fill),
            Cell("Active board side", self.board_side, "mm"),
            Cell("Storage slot pitch (same phase-aligned grid)", self.square_size, "mm"),
            Cell("Storage rows each side", self.storage_rows),
            Cell("Storage columns each side", self.storage_columns),
            Cell("Storage width each side", self.storage_width_each, "mm"),
            Cell("Total motor width", self.motor_width, "mm"),
            Cell("Total motor height", self.motor_height, "mm"),
            Cell("Total motor area", self.motor_area, "mm2"),
        ]


class CoilBed:
    def __init__(self, board):
        short_pitch = Fraction(str(board.period_length)) / Inputs.coils_per_period
        long_pitch = Fraction(str(Inputs.coil_outer_length))
        self.outer_width = float(short_pitch)
        self.outer_length = Inputs.coil_outer_length
        denominator = lcm(short_pitch.denominator, long_pitch.denominator)
        self.lattice_repeat = lcm(int(short_pitch * denominator), int(long_pitch * denominator)) / denominator
        self.columns = Inputs.coils_per_period * Inputs.periods_per_side
        self.rows = ceil(board.platform_side / self.outer_length)
        self.outer_height = 0.0
        self.aspect_ratio = self.outer_length / self.outer_width
        self.bodies_per_orientation = ceil(board.platform_side ** 2 / (self.outer_width * self.outer_length))
        self.bodies_under_platform = Fixed.herringbone_orientation_families * self.bodies_per_orientation
        self.control_cells_per_side = Inputs.control_cells_per_side
        self.control_columns = self.control_cells_per_side
        self.control_rows = max(1, round(self.control_cells_per_side * self.outer_width / self.outer_length))
        self.control_bed_per_orientation = self.control_columns * self.control_rows
        self.control_bed_bodies = Fixed.herringbone_orientation_families * self.control_bed_per_orientation
        self.footprint_area = self.outer_width * self.outer_length
        self.winding_radial_width = Inputs.winding_radial_width
        self.conductor_radial_width = (self.winding_radial_width - Fixed.coil_winding_clearance) / Fixed.turns_per_radial_layer - 2 * Fixed.rectangular_wire_film
        self.body_density = Fixed.herringbone_orientation_families / (self.outer_width * self.outer_length)
        self.coil_spacing = sqrt(1 / self.body_density)
        self.total_bodies = ceil(board.motor_area * self.body_density)
        self.windings = self.total_bodies * Fixed.windings_per_coil_body
        self.flight_active_bodies = self.control_bed_bodies * Inputs.pieces_levitating_simultaneously
        self.active_bodies = min(self.total_bodies, self.control_bed_bodies * Inputs.piece_count)
        self.active_windings = self.active_bodies * Fixed.windings_per_coil_body
        self.peak_driven_windings = ceil(self.active_windings * Inputs.drive_look_ahead_factor)
        self.half_bridges_per_coil = Fixed.driver_half_bridges_per_coil
        self.drive_voltage_fraction = Fixed.coil_bus_voltage_fraction

    def cells(self):
        return [
            Cell("Coil outer width", self.outer_width, "mm"),
            Cell("Coil outer length", self.outer_length, "mm"),
            Cell("Coil outer height", self.outer_height, "mm"),
            Cell("Coil aspect ratio (actual)", self.aspect_ratio),
            Cell("Coil body footprint area", self.footprint_area, "mm2"),
            Cell("Winding radial width (wall)", self.winding_radial_width, "mm"),
            Cell("Conductor radial width", self.conductor_radial_width, "mm"),
            Cell("Coil columns across width", self.columns),
            Cell("Coil rows along length", self.rows),
            Cell("Coil bodies per orientation", self.bodies_per_orientation),
            Cell("Coil bodies under platform", self.bodies_under_platform),
            Cell("6-DOF control bed per orientation", self.control_bed_per_orientation),
            Cell("6-DOF control bed bodies", self.control_bed_bodies),
            Cell("Coil body density", self.body_density, "1/mm2"),
            Cell("Equivalent coil spacing", self.coil_spacing, "mm"),
            Cell("Total coil bodies", self.total_bodies),
            Cell("Total windings (full board)", self.windings),
            Cell("Flight local coil count", self.flight_active_bodies),
            Cell("Reset active-coil upper bound", self.active_bodies),
            Cell("Reset active-winding upper bound", self.active_windings),
            Cell("Peak driven windings (+thrust look-ahead)", self.peak_driven_windings),
            Cell("Dedicated coil driver channels", self.total_bodies),
        ]


class HalbachArray:
    def __init__(self, board):
        self.blocks_per_side = Inputs.periods_per_side * Inputs.magnets_per_period
        self.block_volume = Inputs.magnet_lateral_edge ** 2 * Inputs.magnet_thickness
        self.blocks_per_platform = self.blocks_per_side ** 2
        self.block_mass = self.block_volume * Constants.ndfeb_density
        self.magnet_mass = self.blocks_per_platform * self.block_mass
        self.circumradius = board.platform_side / 2 * sqrt(2)
        self.resting_magnet_gap = board.base_diameter - 2 * self.circumradius

    def cells(self):
        return [
            Cell("Magnet blocks per side", self.blocks_per_side),
            Cell("Magnet block lateral edge", Inputs.magnet_lateral_edge, "mm"),
            Cell("Magnet block thickness", Inputs.magnet_thickness, "mm"),
            Cell("Magnet block volume", self.block_volume, "mm3"),
            Cell("Magnet blocks per platform", self.blocks_per_platform),
            Cell("Magnet block mass", self.block_mass, "g"),
            Cell("Magnet mass total", self.magnet_mass, "g"),
            Cell("Magnet corner reach (circumradius)", self.circumradius, "mm"),
            Cell("Resting magnet gap (bases touching)", self.resting_magnet_gap, "mm"),
        ]


class Piece:
    def plastic_shell_volume(self, diameter, height, wall_thickness):
        return pi / 4 * (diameter ** 2 * height - (diameter - 2 * wall_thickness) ** 2 * (height - 2 * wall_thickness))

    def __init__(self, board, halbach):
        self.scale = board.base_diameter / Fixed.reference_king_base_diameter
        self.box_height = Fixed.reference_king_height * self.scale
        self.diameter = board.base_diameter
        self.shell_envelope_volume = self.plastic_shell_volume(self.diameter, self.box_height, Inputs.plastic_wall_thickness)
        self.shell_volume = self.shell_envelope_volume * Inputs.piece_shell_shape_factor
        self.shell_mass = self.shell_volume * Constants.plastic_density
        self.mass = self.shell_mass + halbach.magnet_mass
        self.magnet_fraction = halbach.magnet_mass / self.mass
        self.weight = self.mass / 1000 * Constants.gravity
        self.shell_com_height = self.box_height / 2000
        self.magnet_com_height = Inputs.magnet_thickness / 2000
        self.com_height = (self.shell_mass * self.shell_com_height + halbach.magnet_mass * self.magnet_com_height) / self.mass
        shell_tilt_inertia = 0
        shell_yaw_inertia = 0
        for sign, inset in ((1, 0), (-1, Inputs.plastic_wall_thickness)):
            radius = (self.diameter / 2 - inset) / 1000
            height = (self.box_height - 2 * inset) / 1000
            cylinder_mass = pi * radius ** 2 * height * Constants.plastic_density * 1000000 * Inputs.piece_shell_shape_factor
            shell_tilt_inertia += sign * cylinder_mass * (3 * radius ** 2 + height ** 2) / 12
            shell_yaw_inertia += sign * cylinder_mass * radius ** 2 / 2
        magnet_mass = halbach.magnet_mass / 1000
        magnet_side = board.platform_side / 1000
        magnet_height = Inputs.magnet_thickness / 1000
        self.tilt_inertia = (shell_tilt_inertia + self.shell_mass / 1000 * (self.shell_com_height - self.com_height) ** 2
                             + magnet_mass * ((magnet_side ** 2 + magnet_height ** 2) / 12
                                              + (self.magnet_com_height - self.com_height) ** 2))
        self.yaw_inertia = shell_yaw_inertia + magnet_mass * magnet_side ** 2 / 6

    def cells(self):
        return [
            Cell("Size scale", self.scale),
            Cell("Box height (scaled king)", self.box_height, "mm"),
            Cell("Cylinder diameter", self.diameter, "mm"),
            Cell("Hollow bounding-cylinder shell volume", self.shell_envelope_volume, "mm3"),
            Cell("Uniform shell mass/inertia shape factor (approximation)", Inputs.piece_shell_shape_factor),
            Cell("Modelled plastic shell volume", self.shell_volume, "mm3"),
            Cell("Plastic shell mass", self.shell_mass, "g"),
            Cell("King mass (shell + bottom magnets approximation)", self.mass, "g"),
            Cell("Magnet mass fraction", self.magnet_fraction),
            Cell("Piece weight", self.weight, "N"),
            Cell("COM above magnet bottom", self.com_height * 1000, "mm"),
            Cell("Roll/pitch inertia about COM", self.tilt_inertia, "kg.m2"),
            Cell("Yaw inertia about COM", self.yaw_inertia, "kg.m2"),
        ]


class NeighbourSnap:
    def __init__(self, board, piece, halbach, sim):
        self.center_distance = board.base_diameter
        self.corner_gap = board.base_diameter - 2 * halbach.circumradius
        self.snap_force = sim["neighbour_snap_force"]
        self.snap_to_weight = self.snap_force / piece.weight
        self.holding_friction = Fixed.resting_friction_coefficient

    def cells(self):
        return [
            Cell("Bases-touching center distance", self.center_distance, "mm"),
            Cell("Worst-orientation magnet corner gap", self.corner_gap, "mm"),
            Cell("Neighbour snap force (worst orientation)", self.snap_force * 1000, "mN"),
            Cell("Snap-to-weight (needed friction)", self.snap_to_weight),
            Cell("Resting friction available", self.holding_friction),
        ]


class WindingGeometry:
    def __init__(self, coil, conductor_thickness, layers):
        self.wire = rectangular_wire(coil.conductor_radial_width, conductor_thickness)
        self.turns_per_layer = Fixed.turns_per_radial_layer
        self.radial_width = coil.winding_radial_width
        self.layers = layers
        self.turns = self.turns_per_layer * self.layers
        self.inner_window_width = coil.outer_width - 2 * self.radial_width
        self.inner_window_length = coil.outer_length - 2 * self.radial_width
        self.coil_height = self.layers * self.wire.axial_pitch
        self.average_length_per_turn = 2 * ((coil.outer_length - self.radial_width) + (coil.outer_width - self.radial_width))
        self.length_per_winding = self.turns * self.average_length_per_turn / 1000
        self.cross_section_area = self.wire.copper_area
        self.resistance = Constants.copper_resistivity * self.length_per_winding / self.cross_section_area


class CoilConfiguration(WindingGeometry):
    def __init__(self, coil, piece, sim, conductor_thickness, bus_voltage, layers):
        super().__init__(coil, conductor_thickness, layers)
        self.bus_voltage = bus_voltage
        self.height_coupling = interpolate_height_coupling(sim, self.coil_height)
        self.usable_drive_voltage = bus_voltage * coil.drive_voltage_fraction * Fixed.usable_bus_voltage_fraction
        self.voltage_limited_current = self.usable_drive_voltage / self.resistance
        self.current_limit = min(self.voltage_limited_current, Fixed.driver_channel_current)
        self.force_per_amp = self.turns * sim["lift_force_per_ampere_turn"] * self.height_coupling
        self.available_force = self.current_limit * self.force_per_amp
        self.available_margin = self.available_force / piece.weight
        self.required_current = piece.weight * Inputs.force_safety_factor / self.force_per_amp
        self.best_phase_hover_power = self.resistance * sim["hover_ampere_turns_squared_sum"] / self.turns ** 2 / self.height_coupling ** 2
        self.piece_hover_power = self.resistance * sim["average_hover_ampere_turns_squared_sum"] / self.turns ** 2 / self.height_coupling ** 2
        self.worst_force_per_amp = self.turns * sim["worst_lift_force_per_ampere_turn"] * self.height_coupling
        self.worst_available_force = self.current_limit * self.worst_force_per_amp
        self.worst_available_margin = self.worst_available_force / piece.weight
        self.worst_required_current = piece.weight * Inputs.force_safety_factor / self.worst_force_per_amp
        self.worst_piece_hover_power = self.resistance * sim["worst_hover_ampere_turns_squared_sum"] / self.turns ** 2 / self.height_coupling ** 2
        self.showpiece_force_per_amp = self.turns * sim["showpiece_lift_force_per_ampere_turn"] * self.height_coupling
        self.showpiece_available_margin = self.current_limit * self.showpiece_force_per_amp / piece.weight
        self.showpiece_hover_power = self.resistance * sim["showpiece_hover_ampere_turns_squared_sum"] / self.turns ** 2 / self.height_coupling ** 2
        sprint_sum, = sim["cruise_ampere_turns_squared_sums"]
        worst_sprint_sum, = sim["worst_cruise_ampere_turns_squared_sums"]
        sprint_peak, = sim["cruise_peak_ampere_turns"]
        self.sprint_piece_power = self.resistance * sprint_sum / self.turns ** 2 / self.height_coupling ** 2
        self.worst_sprint_piece_power = self.resistance * worst_sprint_sum / self.turns ** 2 / self.height_coupling ** 2
        self.sprint_current_margin = self.turns * self.current_limit / (sprint_peak / self.height_coupling)


        self.touch_cap_resistance_factor = 1 + Constants.copper_resistivity_tempco * (Fixed.max_touch_temperature - Fixed.material_reference_temperature)
        self.touch_cap_remanence_factor = 1 + Constants.ndfeb_br_tempco * (Fixed.max_touch_temperature - Fixed.material_reference_temperature)
        self.touch_cap_current_limit = min(
            Fixed.driver_channel_current,
            self.usable_drive_voltage / (self.resistance * self.touch_cap_resistance_factor),
        )
        self.hot_sprint_current_margin = (
            self.turns * self.touch_cap_current_limit * self.touch_cap_remanence_factor
            / (sprint_peak / self.height_coupling)
        )
        self.worst_case_poses = sim["worst_case_poses"]
        self.worst_case_max_gap = sim["worst_case_max_gap"] * 1000
        self.worst_case_max_tilt_deg = sim["worst_case_max_tilt_deg"]
        self.total_force = self.required_current * self.force_per_amp
        self.force_per_body = self.total_force / coil.bodies_under_platform
        self.margin = self.total_force / piece.weight

    def cells(self):
        return [
            Cell("Selected wire", self.wire.label),
            Cell("Winding radial pitch", self.wire.radial_pitch, "mm"),
            Cell("Winding axial pitch", self.wire.axial_pitch, "mm"),
            Cell("Copper cross-section area", self.cross_section_area * 1000000, "mm2"),
            Cell("Selected bus voltage", self.bus_voltage, "V"),
            Cell("MOSFET voltage headroom", Fixed.mosfet_voltage_rating / self.bus_voltage, "x"),
            Cell("Usable drive voltage (per coil)", self.usable_drive_voltage, "V"),
            Cell("Winding radial width", self.radial_width, "mm"),
            Cell("Turns per radial layer", self.turns_per_layer),
            Cell("Vertical wire layers", self.layers),
            Cell("Turns per winding (fits window)", self.turns),
            Cell("Coil height", self.coil_height, "mm"),
            Cell("Coil-height field coupling", self.height_coupling, "x"),
            Cell("Wire length per winding", self.length_per_winding, "m"),
            Cell("Resistance per winding", self.resistance, "ohm"),
            Cell("Voltage-limited current", self.voltage_limited_current, "A"),
            Cell("Driver-limited current", self.current_limit, "A"),
            Cell("Lift force per amp (sim)", self.force_per_amp, "N/A"),
            Cell("Available lift force (at limit)", self.available_force, "N"),
            Cell("Available lift margin", self.available_margin, "x"),
            Cell("Worst-case poses swept", self.worst_case_poses),
            Cell("Worst-case maximum flight gap", self.worst_case_max_gap, "mm"),
            Cell("Worst-case max tilt swept", self.worst_case_max_tilt_deg, "deg"),
            Cell("Worst-case lift force per amp (sim)", self.worst_force_per_amp, "N/A"),
            Cell("Worst-case required lift current", self.worst_required_current, "A"),
            Cell("Worst-case available lift margin", self.worst_available_margin, "x"),
            Cell("Worst-case hover power (one piece)", self.worst_piece_hover_power, "W"),
            Cell("Showpiece-tilt lift margin (deepest tilt)", self.showpiece_available_margin, "x"),
            Cell("Showpiece-tilt hover power (one piece)", self.showpiece_hover_power, "W"),
            Cell("Sprint power, full-diagonal A (one piece, phase avg)", self.sprint_piece_power, "W"),
            Cell("Sprint power (worst level pose)", self.worst_sprint_piece_power, "W"),
            Cell("Sprint peak coil demand vs driver budget", self.sprint_current_margin, "x"),
            Cell(f"Hot sprint current limit ({Fixed.max_touch_temperature:g}C winding)", self.touch_cap_current_limit, "A"),
            Cell(f"Hot sprint margin ({Fixed.max_touch_temperature:g}C magnet/winding)", self.hot_sprint_current_margin, "x"),
            Cell("Best-phase hover power (one piece)", self.best_phase_hover_power, "W"),
            Cell("Phase-averaged hover power (one piece)", self.piece_hover_power, "W"),
            Cell("Operating current per winding", self.required_current, "A"),
            Cell("Lift force per coil body", self.force_per_body, "N"),
            Cell("Total lift force", self.total_force, "N"),
            Cell("Lift margin (with safety)", self.margin, "x"),
        ]


def selected_winding(coil, piece, measurements):
    return CoilConfiguration(coil, piece, measurements, Inputs.selected_conductor_thickness,
                             Inputs.selected_bus_voltage, Inputs.selected_winding_layers)


class ConfigurationSweep:
    def __init__(self, coil, piece, sim):
        self.configurations = [
            CoilConfiguration(coil, piece, sim, thickness, bus_voltage, layers)
            for bus_voltage in Inputs.standard_bus_voltages
            for thickness in Inputs.winding_conductor_thicknesses
            for layers in range(1, floor(coil.outer_width / (thickness + 2 * Fixed.rectangular_wire_film)) + 1)
        ]
        self.feasible = [candidate for candidate in self.configurations
                         if self.first_failed_gate(coil, candidate) is None]
        candidates = [candidate for candidate in self.feasible if candidate.bus_voltage == Inputs.selected_bus_voltage]
        if not candidates:
            raise ValueError("No winding meets the flight screening constraints at the selected bus voltage")
        self.selected = min(candidates, key=lambda candidate: candidate.worst_sprint_piece_power)
        self.best_per_voltage = [min([candidate for candidate in self.feasible if candidate.bus_voltage == voltage],
                                    key=lambda candidate: candidate.worst_sprint_piece_power)
                                 for voltage in Inputs.standard_bus_voltages
                                 if any(candidate.bus_voltage == voltage for candidate in self.feasible)]

    def first_failed_gate(self, coil, candidate):
        if candidate.inner_window_width <= 0 or candidate.inner_window_length <= 0:
            return "window"
        if candidate.wire.radial_pitch * candidate.turns_per_layer + Fixed.coil_winding_clearance > coil.winding_radial_width + Fixed.allocator_limit_tolerance:
            return "insulated_winding_fit"
        if candidate.bus_voltage * Fixed.min_mosfet_voltage_headroom > Fixed.mosfet_voltage_rating:
            return "voltage_rating"
        if candidate.worst_available_margin < Inputs.force_safety_factor:
            return "lift"
        if candidate.hot_sprint_current_margin < 1:
            return "flight_current"
        return None


class WindingInventory:
    def __init__(self, coil, config):
        self.copper_mass = Constants.copper_density * config.length_per_winding * config.cross_section_area * coil.windings


class DiscreteDriver:
    def __init__(self, coil):
        self.channels = coil.total_bodies
        self.half_bridges = self.channels * coil.half_bridges_per_coil
        self.active_channels = coil.active_windings
        self.current_feedback_channels = self.channels
        self.control_bits = self.channels
        self.gate_drivers = ceil(self.half_bridges / Fixed.gate_driver_half_bridges)
        self.serial_clock = min(Fixed.driver_serial_clock, Fixed.shift_register_clock_rating)
        needed_bits = log2(Fixed.driver_channel_current / Fixed.max_current_command_error)
        self.required_oversampling_ratio = 2 ** ((Fixed.setpoint_snr_bits_coefficient * needed_bits + Fixed.setpoint_snr_offset) / Fixed.setpoint_snr_oversampling_coefficient)
        output_bit_rate_per_channel = 2 * Fixed.setpoint_filter_cutoff * self.required_oversampling_ratio
        self.max_channels_per_lane = floor(
            self.serial_clock / (output_bit_rate_per_channel * Fixed.min_setpoint_serial_headroom)
        )
        self.setpoint_lane_count = ceil(self.control_bits / self.max_channels_per_lane)
        self.control_bits_per_lane = ceil(self.control_bits / self.setpoint_lane_count)
        self.serialized_slots = self.control_bits_per_lane * self.setpoint_lane_count
        self.shift_registers_per_lane = ceil(self.control_bits_per_lane / Fixed.shift_register_outputs)
        self.shift_registers = self.shift_registers_per_lane * self.setpoint_lane_count
        self.setpoint_frame_rate = self.serial_clock / self.control_bits_per_lane
        self.setpoint_oversampling_ratio = self.setpoint_frame_rate / (2 * Fixed.setpoint_filter_cutoff)
        self.effective_setpoint_bits = min(Fixed.max_setpoint_bits, (Fixed.setpoint_snr_oversampling_coefficient * log2(self.setpoint_oversampling_ratio) - Fixed.setpoint_snr_offset) / Fixed.setpoint_snr_bits_coefficient)
        self.setpoint_resolution = Fixed.driver_channel_current / 2 ** self.effective_setpoint_bits
        self.required_serial_rate_per_lane = self.control_bits_per_lane * output_bit_rate_per_channel
        self.output_serial_headroom = self.serial_clock / self.required_serial_rate_per_lane
        self.command_data_rate = self.control_bits * Fixed.max_setpoint_bits * Fixed.current_command_rate
        self.command_link_headroom = Fixed.driver_serial_clock / self.command_data_rate
        self.serial_headroom = min(self.output_serial_headroom, self.command_link_headroom)
        self.current_offset_error = Fixed.current_sense_offset_residual / Fixed.current_sense_resistance
        self.control_power = (
            self.shift_registers
            * Fixed.shift_register_power_capacitance
            * Fixed.logic_gate_voltage ** 2
            * self.serial_clock
        )

    def cells(self):
        return [
            Cell("Driver implementation", "current-regulated N-MOSFET half-bridges, split rail"),
            Cell("Driver control implementation", "one logical-board FPGA command engine + parallel delta-sigma lanes; local midpoint-referenced comparator loops, idle zero-calibrated"),
            Cell("Dedicated driver channels", self.channels),
            Cell("Current feedback channels", self.current_feedback_channels),
            Cell("Discrete half-bridge legs", self.half_bridges),
            Cell("Gate-driver ICs", self.gate_drivers),
            Cell("Driver cycle-average current capability", Fixed.driver_channel_current, "A"),
            Cell("Driver instantaneous peak-current target", Fixed.driver_peak_current, "A"),
            Cell("MOSFET voltage rating", Fixed.mosfet_voltage_rating, "V"),
            Cell("Comparator current-offset error", self.current_offset_error * 1000, "mA"),
            Cell("MOSFET gate drive", Fixed.gate_drive_voltage, "V"),
            Cell("PWM frequency", Fixed.driver_pwm_frequency / 1000, "kHz"),
            Cell("Current-regulator update rate", Fixed.current_regulator_rate / 1000, "kHz"),
            Cell("Setpoint frame rate (delta-sigma)", self.setpoint_frame_rate / 1000, "kHz"),
            Cell("Setpoint oversampling ratio", self.setpoint_oversampling_ratio, "x"),
            Cell("Effective setpoint bits (delta-sigma)", self.effective_setpoint_bits),
            Cell("Current command refresh", Fixed.current_command_rate, "Hz"),
            Cell("Current loop bandwidth limit", Fixed.current_loop_bandwidth, "Hz"),
            Cell("Comparator offset (raw / after zero-cal)", f"{Fixed.comparator_input_offset * 1000:g} / {Fixed.current_sense_offset_residual * 1000:g} mV"),
            Cell("Parallel setpoint-output lanes", self.setpoint_lane_count),
            Cell("Channels per setpoint lane", self.control_bits_per_lane),
            Cell("Serialized slots incl. lane padding", self.serialized_slots),
            Cell("Setpoint-output clock per lane", self.serial_clock / 1000000, "Mbit/s"),
            Cell("Required output rate per lane", self.required_serial_rate_per_lane / 1000000, "Mbit/s"),
            Cell("Setpoint-output serial headroom", self.output_serial_headroom, "x"),
            Cell("Board command data to FPGA", self.command_data_rate / 1000000, "Mbit/s"),
            Cell("Board command-link headroom", self.command_link_headroom, "x"),
            Cell("Minimum serial headroom", self.serial_headroom, "x"),
            Cell("Pessimistic hot path resistance", Fixed.driver_hot_resistance, "ohm"),
            Cell("Current shunt resistance", Fixed.current_sense_resistance, "ohm"),
            Cell("Shift-register dynamic loss", self.control_power, "W"),
            Cell("Serialized control bits", self.control_bits),
            Cell("74HC595 shift registers", self.shift_registers),
        ]


class RadiatorCooling:
    def __init__(self, board, config, fan_count, grind_baseline, scenarios, architecture):
        self.material_failures = []
        self.fan_count = fan_count
        self.mode = "fan-assisted replay" if grind_baseline else "passive play"
        self.board_area = board.motor_area / 1000000
        self.piece_footprint_area = pi / 4 * (board.base_diameter / 1000) ** 2
        self.coil_bed_thickness = Fixed.herringbone_orientation_families * config.coil_height
        self.stack_area_resistance = (
            self.coil_bed_thickness / 2000 / Fixed.coil_bed_through_conductivity
            + Fixed.potting_thickness / 1000 / Fixed.potting_thermal_conductivity
            + Fixed.pcb_thickness / 1000 / Fixed.pcb_via_effective_thermal_conductivity
            + Fixed.radiator_standoff_below_pcb / 1000 / Fixed.thermal_pad_conductivity
            + Fixed.baseplate_thickness / 1000 / Fixed.aluminium_thermal_conductivity)
        self.local_resistance = self.stack_area_resistance / self.piece_footprint_area
        self.source_time_constant = self.stack_area_resistance * self.coil_bed_thickness / 1000 * Fixed.potting_volumetric_heat_capacity
        fin_pitch = (Fixed.fin_thickness + Fixed.fin_channel_width) / 1000
        self.fin_count = floor((board.motor_width + Fixed.fin_channel_width) / 1000 / fin_pitch)
        self.convection_area = self.board_area + 2 * self.fin_count * Fixed.fin_height / 1000 * board.motor_height / 1000
        convection = Fixed.forced_convection_coefficient if fan_count else Fixed.natural_convection_coefficient
        self.thermal_conductance = convection * self.convection_area
        volume = self.board_area * Fixed.baseplate_thickness / 1000 + self.fin_count * Fixed.fin_thickness / 1000 * Fixed.fin_height / 1000 * board.motor_height / 1000
        self.aluminium_mass = volume * Fixed.aluminium_density
        self.thermal_capacitance = self.aluminium_mass * Fixed.aluminium_heat_capacity
        self.thermal_time_constant = self.thermal_capacitance / self.thermal_conductance
        self.fan_power = fan_count * Fixed.cooling_fan_power
        self.background_power = architecture.electronics_power + self.fan_power
        move_period = 60 / Inputs.sustained_moves_per_minute if grind_baseline else Inputs.play_move_period
        self.sustained_power = scenarios.composite_energy / move_period
        self.sustained_plate_temperature = self.steady_temperature(self.sustained_power)
        self.baseline_rise = self.sustained_plate_temperature - Inputs.ambient_temperature
        reset_duty = scenarios.reset_energy / Inputs.sustained_event_period
        periodic_baseline = self.steady_temperature(reset_duty)
        self.event_segments = scenarios.reset_segments * Inputs.events_back_to_back
        self.event_duration = sum(duration for duration, _, _ in scenarios.reset_segments)
        self.event_energy = scenarios.reset_energy
        self.reset_peak_plate_temperature, self.cyclic_peak_source_temp = self.trace_peak(self.event_segments, periodic_baseline)
        self.cyclic_peak_baseplate_temp = max(self.reset_peak_plate_temperature, self.sustained_plate_temperature)
        hammer = scenarios.hover_worst_power
        on_decay = exp(-Inputs.hammer_dwell / self.source_time_constant)
        period_decay = exp(-Inputs.hammer_visit_period / self.source_time_constant)
        hammer_peak = hammer * self.local_resistance * (1 - on_decay) / (1 - period_decay)
        hammer_baseline = self.steady_temperature(hammer * Inputs.hammer_dwell / Inputs.hammer_visit_period)
        for _ in range(Fixed.thermal_max_iterations):
            corrected_peak = hammer * self.hot_power_factor(hammer_baseline + hammer_peak) * self.local_resistance * (1 - on_decay) / (1 - period_decay)
            if hammer_baseline + corrected_peak > Fixed.magnet_max_operating_temperature:
                self.material_failures.append("Periodic local heating exceeds the magnet temperature range")
                hammer_peak = float("inf")
                break
            if abs(corrected_peak - hammer_peak) <= Fixed.thermal_convergence_tolerance:
                hammer_peak = corrected_peak
                break
            hammer_peak = corrected_peak
        else:
            raise ValueError("Periodic local thermal model did not converge")
        cascade = [(Inputs.cascade_exchanges * 2 * Inputs.hammer_dwell, hammer, hammer)]
        cascade.extend(scenarios.flight_segments)
        _, self.worst_piece_local_temp = self.trace_peak(cascade, hammer_baseline, hammer_peak)
        self.peak_cell_temperature = max(self.cyclic_peak_source_temp, self.worst_piece_local_temp, self.sustained_plate_temperature)
        self.fan_mass = fan_count * Fixed.cooling_fan_mass
        self.fan_bank_fits = fan_count * Fixed.cooling_fan_size <= board.motor_width and (not fan_count or Fixed.cooling_fan_size <= board.motor_height)
        self.fan_noise = Fixed.cooling_fan_noise + 10 * log10(fan_count) + Fixed.cooling_fan_installation_noise if fan_count else 0

    def hover_endurance(self, power):
        allowed_rise = Fixed.max_touch_temperature - Inputs.ambient_temperature - self.baseline_rise
        if allowed_rise <= 0:
            return 0.0
        steady_rise = power * self.hot_power_factor(Fixed.max_touch_temperature) * self.local_resistance
        if steady_rise <= allowed_rise:
            return float("inf")
        return -self.source_time_constant * log(1 - allowed_rise / steady_rise)

    def steady_temperature(self, motion_power):
        temperature = Inputs.ambient_temperature
        for _ in range(Fixed.thermal_max_iterations):
            updated = Inputs.ambient_temperature + (motion_power * self.hot_power_factor(temperature) + self.background_power) / self.thermal_conductance
            if updated > Fixed.magnet_max_operating_temperature:
                self.material_failures.append("No steady equilibrium within the magnet temperature range")
                return float("inf")
            if abs(updated - temperature) <= Fixed.thermal_convergence_tolerance:
                return updated
            temperature = updated
        raise ValueError("Steady thermal model did not converge")

    def hot_power_factor(self, temperature):
        reference_difference = temperature - Fixed.material_reference_temperature
        remanence_factor = 1 + Constants.ndfeb_br_tempco * reference_difference
        if temperature > Fixed.magnet_max_operating_temperature or remanence_factor <= 0:
            raise ValueError("Thermal trajectory exceeds magnet material model")
        return (1 + Constants.copper_resistivity_tempco * reference_difference) / remanence_factor ** 2

    def trace_peak(self, segments, initial_plate_temperature, initial_local_rise=0.0):
        if not isfinite(initial_plate_temperature) or not isfinite(initial_local_rise):
            return float("inf"), float("inf")
        plate = initial_plate_temperature
        local_rise = initial_local_rise
        peak_plate = plate
        peak_source = plate + local_rise
        for duration, total_power, local_power in segments:
            steps = ceil(duration / Fixed.thermal_trace_step)
            timestep = duration / steps
            for _ in range(steps):
                factor = self.hot_power_factor(plate + local_rise)
                plate_equilibrium = Inputs.ambient_temperature + (total_power * factor + self.background_power) / self.thermal_conductance
                plate = plate_equilibrium + (plate - plate_equilibrium) * exp(-timestep / self.thermal_time_constant)
                local_equilibrium = local_power * factor * self.local_resistance
                local_rise = local_equilibrium + (local_rise - local_equilibrium) * exp(-timestep / self.source_time_constant)
                peak_plate = max(peak_plate, plate)
                peak_source = max(peak_source, plate + local_rise)
                if peak_source > Fixed.magnet_max_operating_temperature:
                    self.material_failures.append("Transient exceeds magnet material range; integration stopped")
                    return peak_plate, peak_source
        return peak_plate, peak_source

    def cells(self):
        return [
            Cell("Cooling mode", self.mode),
            Cell("Sustained motion power (cold estimate)", self.sustained_power, "W"),
            Cell("Contact reset duration", self.event_duration, "s"),
            Cell("Contact reset energy (cold estimate)", self.event_energy, "J"),
            Cell("Back-to-back trace segments", len(self.event_segments)),
            Cell("T1 reset plate temperature (screen)", self.reset_peak_plate_temperature, "C"),
            Cell("T2 sustained plate temperature (inf = infeasible)", self.sustained_plate_temperature, "C"),
            Cell("Material-model failures", "; ".join(self.material_failures) if self.material_failures else "none"),
            Cell("Peak reset cell temperature (stationary-footprint bound)", self.cyclic_peak_source_temp, "C"),
            Cell("Hammer/cascade cell temperature (screen)", self.worst_piece_local_temp, "C"),
            Cell("Worst screened cell temperature", self.peak_cell_temperature, "C"),
            Cell("Radiator mass", self.aluminium_mass, "kg"),
            Cell("Fans", self.fan_count),
            Cell("Fan speed metadata", Fixed.cooling_fan_speed, "rpm"),
            Cell("Fan pressure metadata", Fixed.cooling_fan_static_pressure, "mm H2O"),
            Cell("Effective airflow assumption", self.fan_count * Fixed.cooling_fan_airflow * Fixed.cooling_fan_airflow_fraction, "m3/h"),
            Cell("Installed fan noise estimate", self.fan_noise, "dB(A)"),
            Cell("Reset model limitation", "isolated-piece sum; shared coils and routing require verification"),
        ]


class SurfaceStack:
    def __init__(self, thermal):
        self.cover = Fixed.potting_cover_thickness
        self.surface = Fixed.playing_surface_thickness
        self.skin = Fixed.piece_bottom_skin
        self.stack = self.cover + self.surface + self.skin
        self.visible_hover = Inputs.magnet_to_coil_distance - self.stack
        self.visible_flight = Inputs.max_flight_gap - self.stack
        self.flatness_fraction = Fixed.surface_flatness_budget / self.visible_hover
        conduction_resistance = (self.cover / 1000 / Fixed.potting_thermal_conductivity
                                 + self.surface / 1000 / Fixed.playing_surface_conductivity)
        convection_resistance = 1 / Fixed.natural_convection_coefficient
        self.hotspot_touch_temperature = Inputs.ambient_temperature + (
            (thermal.peak_cell_temperature - Inputs.ambient_temperature)
            * convection_resistance / (conduction_resistance + convection_resistance))
        self.idle_touch_temperature = Inputs.ambient_temperature + thermal.baseline_rise
        self.idle_peak_temperature = thermal.cyclic_peak_baseplate_temp

    def cells(self):
        return [
            Cell("Potting cover over coils", self.cover, "mm"),
            Cell("Playing surface thickness", self.surface, "mm"),
            Cell("Piece bottom skin", self.skin, "mm"),
            Cell("Total top stack", self.stack, "mm"),
            Cell("Visible hover height (nominal)", self.visible_hover, "mm"),
            Cell("Visible hover height (max flight)", self.visible_flight, "mm"),
            Cell("Surface flatness budget", Fixed.surface_flatness_budget, "mm"),
            Cell("Flatness fraction of visible hover", self.flatness_fraction, "x"),
            Cell("Hotspot surface temp (worst cell, ever)", self.hotspot_touch_temperature, "C"),
            Cell("Idle-region surface temp (sustained, prolonged touch)", self.idle_touch_temperature, "C"),
            Cell("Idle-region surface temp (rare double-reset peak)", self.idle_peak_temperature, "C"),
            Cell("Touch policy", "every cell must pass temperature limits; no governor credit is modeled"),
        ]


def split_rail_ripple_peak(resistance, inductance, rail_voltage, frequency):
    decay_exponent = resistance / (inductance * frequency)
    period_decay_loss = -expm1(-decay_exponent)
    extremum_decay = period_decay_loss / decay_exponent
    extremum_duty = -log(extremum_decay) / decay_exponent
    return 2 * rail_voltage / resistance * (
        1 - extremum_duty - (extremum_decay - exp(-decay_exponent)) / period_decay_loss)


class Control:
    def __init__(self, coil, config, allocations):
        self.inductance = Constants.vacuum_permeability * config.turns ** 2 * coil.footprint_area / 1000000 / (config.coil_height / 1000)
        self.minimum_inductance = self.inductance * min(Fixed.qualification_inductance_factors)
        self.current_ripple_peak = split_rail_ripple_peak(
            config.resistance, self.minimum_inductance, config.bus_voltage * Fixed.coil_bus_voltage_fraction,
            Fixed.driver_pwm_frequency)
        self.switch_current_bound = config.current_limit + self.current_ripple_peak
        self.electrical_bandwidth = config.resistance / (2 * pi * self.inductance)
        self.actuator_bandwidth = min(self.electrical_bandwidth, Fixed.current_loop_bandwidth)
        self.slew_time = self.inductance * config.current_limit / config.usable_drive_voltage * 1000
        self.growth_rate = allocations["instability_growth_rate"]
        if self.growth_rate <= 0:
            raise ValueError("No unstable mode found; rate-selection method needs an explicit stable-plant model")
        self.required_bandwidth = Inputs.control_loop_bandwidth_margin * self.growth_rate / (2 * pi)
        self.pose_rate_screen = Inputs.control_loop_bandwidth_margin * self.growth_rate
        self.pose_update_rate = Fixed.pose_feedback_rate

    def cells(self):
        return [
            Cell("Selected-winding inductance (screening estimate)", self.inductance * 1000, "mH"),
            Cell("Assumed minimum self-inductance", self.minimum_inductance * 1000, "mH"),
            Cell("PWM steady-cycle ripple peak deviation (bound)", self.current_ripple_peak, "A"),
            Cell("Peak switch current at full average rating (screen)", self.switch_current_bound, "A"),
            Cell("Switching limitation", "periodic independent RL screen; measured inductance, sensing, dead time and rail transients unverified"),
            Cell("Nominal six-axis unstable growth rate", self.growth_rate, "1/s"),
            Cell("Pose rate from instability margin (not validated minimum)", self.pose_rate_screen, "Hz"),
            Cell("Candidate pose feedback rate", self.pose_update_rate, "Hz"),
            Cell("Required bandwidth screening target", self.required_bandwidth, "Hz"),
            Cell("Estimated actuator bandwidth", self.actuator_bandwidth, "Hz"),
            Cell("Current slew to driver limit (ideal voltage step)", self.slew_time, "ms"),
            Cell("Control limitation", "see qualification.json for scoped dynamic tests; complete flight and neighbour validation outstanding"),
        ]


class DriveMatrix:
    def __init__(self, control, driver, architecture):
        self.control = control
        self.driver = driver
        self.architecture = architecture
        self.current_command_rate_headroom = Fixed.current_command_rate / control.pose_update_rate
        self.current_loop_headroom = control.actuator_bandwidth / control.required_bandwidth
        self.current_regulator_samples = Fixed.current_regulator_rate / Fixed.current_loop_bandwidth

    def cells(self):
        return [
            Cell("Current command-rate headroom", self.current_command_rate_headroom, "x"),
            Cell("Current bandwidth screening headroom", self.current_loop_headroom, "x"),
            Cell("Current-regulator samples per loop bandwidth", self.current_regulator_samples),
            Cell("PWM periods per current-regulator update", Fixed.driver_pwm_frequency / Fixed.current_regulator_rate),
            Cell("Current command resolution", self.driver.setpoint_resolution * 1000, "mA"),
            Cell("Serial headroom", self.driver.serial_headroom, "x"),
            Cell("Current-monitor headroom", self.architecture.current_monitor_headroom, "x"),
            Cell("Control update period", 1000 / self.control.pose_update_rate, "ms"),
        ]


class HallSensing:
    def __init__(self, coil, config, control, architecture):
        self.update_rate = control.pose_update_rate
        self.sensor_pitch = architecture.hall_pitch
        self.sensors_per_piece = Fixed.hall_observation_window_side ** 2
        self.sensors_per_node = architecture.hall_sensors_per_node
        self.total_sensors = architecture.hall_total_sensors
        self.muxes_per_node = architecture.hall_muxes_per_node
        self.total_muxes = architecture.hall_muxes
        self.oversampling_factor = architecture.hall_oversampling_factor
        self.reads_per_node = self.sensors_per_node * self.update_rate * self.oversampling_factor
        self.node_capacity = Fixed.control_node_adc_engines * Fixed.hall_adc_sample_rate
        self.headroom = architecture.hall_scan_headroom
        self.averaging_group_delay_ms = 0.5 * self.oversampling_factor / Fixed.hall_adc_sample_rate * 1000
        verified = levitation_sim.verified_hall_sensing(coil.control_cells_per_side, config.coil_height / 1000,
                                                        config.turns * config.current_limit, self.sensor_pitch / 1000)
        self.plane_depth = verified["plane_depth_below_magnets"] * 1000
        self.signal_peak = verified["signal_peak"]
        self.coil_field_hover = verified["coil_field_hover"]
        self.coil_field_hover_bound = verified["coil_field_hover_bound"]
        self.coil_field_budget_bound = verified["coil_field_budget_bound"]
        self.neighbour_field = verified["neighbour_field"]
        self.nominal_rank = verified["rank6"]
        self.nominal_condition = verified["condition6"]
        self.worst_rank = verified["worst_rank6"]
        self.worst_condition = verified["worst_condition6"]
        self.worst_poses = verified["worst_poses"]
        self.position_noise_gain = verified["worst_position_noise_gain"]
        self.tilt_noise_gain = verified["worst_tilt_noise_gain"]
        self.observation_window_side = Fixed.hall_observation_window_side
        self.adc_quantization_field = Fixed.hall_supply_voltage / 2 ** Fixed.hall_adc_native_bits / Fixed.hall_sensitivity
        self.sensor_noise_averages = max(1.0, min(self.oversampling_factor, self.oversampling_factor * 2 * Fixed.hall_sensor_bandwidth / Fixed.hall_adc_sample_rate))
        self.field_noise = sqrt(Fixed.hall_output_noise ** 2 / self.sensor_noise_averages + self.adc_quantization_field ** 2 / 12 / self.oversampling_factor)
        self.position_noise_um = self.position_noise_gain * self.field_noise * 1e6
        self.tilt_noise_mrad = self.tilt_noise_gain * self.field_noise * 1000
        self.flight_current_scale = sqrt(config.sprint_piece_power / config.piece_hover_power)
        self.interference_bias_um = self.position_noise_gain * (self.coil_field_hover + self.neighbour_field) * Fixed.coil_field_subtraction_error * 1e6
        self.flight_interference_bias_um = self.position_noise_gain * (self.coil_field_hover * self.flight_current_scale + self.neighbour_field) * Fixed.coil_field_subtraction_error * 1e6
        self.total_position_error_um = self.position_noise_um + self.interference_bias_um
        self.flight_position_error_um = self.position_noise_um + self.flight_interference_bias_um
        self.required_position_error_um = Inputs.magnet_to_coil_distance * 1000 * Fixed.position_error_gap_fraction
        self.required_flight_position_error_um = Inputs.magnet_to_coil_distance * 1000 * Fixed.flight_position_error_gap_fraction
        self.saturation_field = self.signal_peak + self.neighbour_field + self.coil_field_hover_bound
        period = coil.outer_width * Inputs.coils_per_period
        surface_stack = Fixed.potting_cover_thickness + Fixed.playing_surface_thickness + Fixed.piece_bottom_skin
        self.rest_descent = Inputs.magnet_to_coil_distance - surface_stack
        self.rest_signal_peak = self.signal_peak * exp(2 * pi * self.rest_descent / period)
        self.rest_saturation_field = self.rest_signal_peak + self.neighbour_field

    def cells(self):
        return [
            Cell("Required update rate", self.update_rate, "Hz"),
            Cell("Hall sensor pitch", self.sensor_pitch, "mm"),
            Cell("Sensor plane depth below magnets", self.plane_depth, "mm"),
            Cell("Estimator observation window", f"{self.observation_window_side}x{self.observation_window_side}"),
            Cell("Sensors used per piece estimate", self.sensors_per_piece),
            Cell("Fixed-grid worst-case poses", self.worst_poses),
            Cell("Nominal Hall observability rank", self.nominal_rank),
            Cell("Nominal Hall condition", self.nominal_condition, "x"),
            Cell("Worst fixed-grid Hall rank", self.worst_rank),
            Cell("Worst fixed-grid Hall condition", self.worst_condition, "x"),
            Cell("Peak magnet signal at sensors", self.signal_peak * 1000, "mT"),
            Cell("Coil field at sensors (hover, signed)", self.coil_field_hover * 1000, "mT"),
            Cell("Coil field bound (hover, worst signs)", self.coil_field_hover_bound * 1000, "mT"),
            Cell("Coil field bound (full driver budget)", self.coil_field_budget_bound * 1000, "mT"),
            Cell("Neighbour piece field at sensors", self.neighbour_field * 1000, "mT"),
            Cell("Worst-case saturation field", self.saturation_field * 1000, "mT"),
            Cell("Rest-pose magnet descent (parked piece)", self.rest_descent, "mm"),
            Cell("Rest-pose saturation field (coils off underneath)", self.rest_saturation_field * 1000, "mT"),
            Cell("Hall linear range", Fixed.hall_linear_range * 1000, "mT"),
            Cell("ADC quantization (field)", self.adc_quantization_field * 1e6, "uT"),
            Cell("Effective field noise per update", self.field_noise * 1e6, "uT"),
            Cell("Worst position noise gain", self.position_noise_gain, "m/T"),
            Cell("Position noise (worst pose)", self.position_noise_um, "um"),
            Cell("Tilt noise (worst pose)", self.tilt_noise_mrad, "mrad"),
            Cell("Interference bias after subtraction (hover currents)", self.interference_bias_um, "um"),
            Cell("Total position error (docking, hover currents)", self.total_position_error_um, "um"),
            Cell("Docking error budget (gap fraction)", self.required_position_error_um, "um"),
            Cell("Flight current scale for bias (sprint/hover)", self.flight_current_scale, "x"),
            Cell("Interference bias after subtraction (sprint currents)", self.flight_interference_bias_um, "um"),
            Cell("Total position error (in transit)", self.flight_position_error_um, "um"),
            Cell("In-transit error budget (gap fraction)", self.required_flight_position_error_um, "um"),
            Cell("Sensors per control node (maximum)", self.sensors_per_node),
            Cell("Total Hall sensors (board)", self.total_sensors),
            Cell("Hall array supply power", self.total_sensors * Fixed.hall_supply_current * Fixed.hall_supply_voltage, "W"),
            Cell("Readout muxes per control node (maximum)", self.muxes_per_node),
            Cell("Total Hall readout muxes", self.total_muxes),
            Cell("ADC oversampling factor", self.oversampling_factor, "x"),
            Cell("Independent sensor-noise averages", self.sensor_noise_averages, "x"),
            Cell("Sensor averaging group delay", self.averaging_group_delay_ms, "ms"),
            Cell("Reads needed per control node (maximum)", self.reads_per_node, "reads/s"),
            Cell("Raw ADC sample capacity per control node", self.node_capacity, "samples/s"),
            Cell("Scan headroom (incl. gating settle)", self.headroom, "x"),
        ]


class BoardControl:
    def __init__(self, board, coil, control, driver, hall_pitch):
        self.architecture = "one logical continuous motor/control board; physical PCB segmentation deferred"
        self.hall_pitch = hall_pitch
        self.motor_width = board.motor_width
        self.motor_height = board.motor_height
        self.motor_area = board.motor_area

        self.hall_sensors_x = ceil(board.motor_width / hall_pitch)
        self.hall_sensors_y = ceil(board.motor_height / hall_pitch)
        self.hall_total_sensors = self.hall_sensors_x * self.hall_sensors_y
        self.hall_muxes = ceil(self.hall_total_sensors / Fixed.hall_sensor_mux_channels)
        self.hall_oversampling_factor = 4 ** max(0, Fixed.hall_interpolation_bits - Fixed.hall_adc_native_bits)
        self.hall_group_burst_time = (
            Fixed.hall_sensor_mux_channels * self.hall_oversampling_factor / Fixed.hall_adc_sample_rate
        )
        self.hall_gating_duty = (
            Fixed.hall_power_settle_time + self.hall_group_burst_time
        ) * control.pose_update_rate
        if self.hall_gating_duty * Fixed.min_hall_scan_headroom > 1:
            raise ValueError("A single Hall mux group cannot be sampled at the required pose rate")
        max_muxes_per_adc = max(1, floor(
            1 / (self.hall_gating_duty * Fixed.min_hall_scan_headroom)
        ))
        self.required_hall_adc_engines = ceil(self.hall_muxes / max_muxes_per_adc)

        self.pose_rate = control.pose_update_rate
        self.node_capacity = Fixed.node_mcu_throughput_mflops * 1e6
        self.piece_compute = Inputs.pieces_levitating_simultaneously * Fixed.piece_control_flops * self.pose_rate
        self.setpoint_stream_compute = coil.total_bodies * Fixed.current_command_rate * Fixed.setpoint_dma_words_flops
        self.total_compute = self.piece_compute + self.setpoint_stream_compute
        compute_nodes = ceil(
            self.total_compute * Fixed.min_control_compute_headroom / self.node_capacity
        )
        hall_nodes = ceil(self.required_hall_adc_engines / Fixed.control_node_adc_engines)
        self.control_node_count = max(1, compute_nodes, hall_nodes)
        while True:
            hall_adc_engines = self.control_node_count * Fixed.control_node_adc_engines
            muxes_per_adc_engine = ceil(self.hall_muxes / hall_adc_engines)
            hall_scan_fraction = muxes_per_adc_engine * self.hall_gating_duty
            driver_channels_per_node = ceil(coil.total_bodies / self.control_node_count)
            current_monitor_reads_per_node = driver_channels_per_node * self.pose_rate
            remaining_adc_capacity = (
                Fixed.control_node_adc_engines
                * Fixed.hall_adc_sample_rate
                * max(0.0, 1 - hall_scan_fraction)
            )
            if (hall_scan_fraction <= 1 / Fixed.min_hall_scan_headroom
                    and remaining_adc_capacity >= current_monitor_reads_per_node):
                break
            self.control_node_count += 1
        self.hall_adc_engines = hall_adc_engines
        self.hall_muxes_per_adc_engine = muxes_per_adc_engine
        self.hall_scan_fraction = hall_scan_fraction
        self.hall_scan_headroom = 1 / self.hall_scan_fraction
        self.hall_sensors_per_node = ceil(self.hall_total_sensors / self.control_node_count)
        self.hall_muxes_per_node = ceil(self.hall_muxes / self.control_node_count)

        self.compute_per_node = self.total_compute / self.control_node_count
        self.compute_headroom = self.node_capacity / self.compute_per_node
        self.single_node_headroom = self.node_capacity / self.total_compute
        self.driver_channels_per_node = driver_channels_per_node
        self.current_monitor_reads_per_node = current_monitor_reads_per_node
        self.current_monitor_adc_capacity = remaining_adc_capacity
        self.current_monitor_headroom = remaining_adc_capacity / current_monitor_reads_per_node
        self.setpoint_fpga_count = Fixed.board_setpoint_fpga_count
        self.setpoint_lane_count = driver.setpoint_lane_count
        self.thermal_sensor_count = ceil(board.motor_area / Fixed.thermal_sensor_area_mm2)
        self.bulk_capacitor_count = ceil(
            coil.total_bodies / Fixed.driver_channels_per_bulk_capacitor
        )
        self.hall_peak_supply_power = (
            self.hall_total_sensors * Fixed.hall_supply_current * Fixed.hall_supply_voltage
        )
        self.hall_supply_power = self.hall_peak_supply_power * self.hall_gating_duty
        self.electronics_power = (self.control_node_count * Fixed.control_node_mcu_power
                                  + self.setpoint_fpga_count * Fixed.board_setpoint_fpga_power
                                  + Fixed.host_power + self.hall_supply_power + driver.control_power)

    def cells(self):
        return [
            Cell("Control architecture", self.architecture),
            Cell("Logical motor/control footprint", f"{self.motor_width:g} x {self.motor_height:g} mm"),
            Cell("Physical PCB partition", "not fixed; one PCB, strips, or panels may implement the same logical board"),
            Cell("Hall pitch (cost-selected, sparsest passing)", self.hall_pitch, "mm"),
            Cell("Hall grid", f"{self.hall_sensors_x} x {self.hall_sensors_y}"),
            Cell("Regional control/ADC nodes", self.control_node_count),
            Cell("Hall ADC engines per node", Fixed.control_node_adc_engines),
            Cell("Total Hall ADC engines", self.hall_adc_engines),
            Cell("Hall mux groups per ADC engine", self.hall_muxes_per_adc_engine),
            Cell("Board-wide setpoint FPGAs", self.setpoint_fpga_count),
            Cell("Parallel setpoint-output lanes", self.setpoint_lane_count),
            Cell("Driver channels per control node", self.driver_channels_per_node),
            Cell("Setpoint-stream compute (board)", self.setpoint_stream_compute / 1e6, "Mflop/s"),
            Cell("Piece-control compute (board)", self.piece_compute / 1e6, "Mflop/s"),
            Cell("Compute load per control node", self.compute_per_node / 1e6, "Mflop/s"),
            Cell("Control-node compute capacity", self.node_capacity / 1e6, "Mflop/s"),
            Cell("Control-node compute headroom", self.compute_headroom, "x"),
            Cell("Single-node compute headroom (information)", self.single_node_headroom, "x"),
            Cell("Hall gating policy", "each mux group is high-side switched and scanned by regional parallel ADC engines"),
            Cell("Thermal sensors budgeted (governor unimplemented)", self.thermal_sensor_count),
            Cell("Hall group burst time", self.hall_group_burst_time * 1000, "ms"),
            Cell("Hall gating duty per mux group", self.hall_gating_duty, "x"),
            Cell("Worst ADC-engine scan fraction", self.hall_scan_fraction, "x"),
            Cell("Hall scan headroom", self.hall_scan_headroom, "x"),
            Cell("Current monitor reads per node", self.current_monitor_reads_per_node, "samples/s"),
            Cell("ADC capacity left for current monitoring", self.current_monitor_adc_capacity, "samples/s per node"),
            Cell("Current-monitor shared-ADC headroom", self.current_monitor_headroom, "x"),
            Cell("Hall supply power (peak, all on)", self.hall_peak_supply_power, "W"),
            Cell("Hall supply power (gated)", self.hall_supply_power, "W"),
        ]


class EnergyBuffer:
    def __init__(self, bus_voltage, zones, deficit_power, deficit_energy):
        self.rail_voltage = bus_voltage / 2
        self.series_cells = ceil(self.rail_voltage / Fixed.supercap_cell_working_voltage)
        self.string_capacitance = Fixed.supercap_cell_capacitance * Fixed.supercap_eol_capacitance_fraction / self.series_cells
        self.string_resistance = Fixed.supercap_cell_esr * Fixed.supercap_eol_esr_factor * self.series_cells
        self.depletion_voltage = self.rail_voltage * (1 - Fixed.bus_droop_fraction / 2)
        self.ir_budget = self.rail_voltage * Fixed.bus_droop_fraction / 2
        self.min_rail_voltage = self.rail_voltage * (1 - Fixed.bus_droop_fraction)
        self.usable_energy_per_string = 0.5 * self.string_capacitance * (self.rail_voltage ** 2 - self.depletion_voltage ** 2)
        self.max_string_current = self.ir_budget / self.string_resistance
        self.rail_deficit_power = deficit_power / 2
        self.rail_deficit_energy = deficit_energy / 2
        strings_for_energy = ceil(self.rail_deficit_energy / self.usable_energy_per_string)
        strings_for_power = ceil(self.rail_deficit_power / self.min_rail_voltage / self.max_string_current)
        self.strings_per_rail = max(strings_for_energy, strings_for_power)
        self.cell_count = 2 * self.strings_per_rail * self.series_cells
        self.usable_energy = 2 * self.strings_per_rail * self.usable_energy_per_string
        self.peak_power = 2 * self.strings_per_rail * self.max_string_current * self.min_rail_voltage
        self.oring_count = 4 * zones if self.cell_count else 0
        self.management_count = 2 if self.cell_count else 0
        self.price = (self.cell_count * (Fixed.supercap_cell_price + Fixed.supercap_balancer_price_per_cell)
                      + self.oring_count * Fixed.supercap_oring_price
                      + self.management_count * Fixed.supercap_management_price_per_rail)
        self.mass = self.cell_count * Fixed.supercap_cell_mass_kg

    def cells(self):
        return [
            Cell("Buffer topology", "one protected supercap bank per rail polarity around the controlled midpoint; four high-current charge/discharge protection paths"),
            Cell("Supercap cell", f"{Fixed.supercap_cell_capacitance:g}F {Fixed.supercap_cell_max_voltage:g}V, run at {Fixed.supercap_cell_working_voltage:g}V; sized at EOL ({Fixed.supercap_eol_capacitance_fraction:.0%} C, {Fixed.supercap_eol_esr_factor:g}x ESR)"),
            Cell("Cells in series per string", self.series_cells),
            Cell("Parallel strings per rail", self.strings_per_rail),
            Cell("Total supercap cells (both rails)", self.cell_count),
            Cell("Bank capacitance per rail", self.strings_per_rail * self.string_capacitance, "F"),
            Cell("Allowed rail droop during burst", Fixed.bus_droop_fraction * self.rail_voltage, "V"),
            Cell("Usable buffer energy", self.usable_energy, "J"),
            Cell("Buffer peak power (ESR-limited)", self.peak_power, "W"),
            Cell("Buffer mass", self.mass, "kg"),
            Cell("Buffer cost (cells + balancing + ORing)", self.price, "USD"),
        ]


class PowerSupply:
    def __init__(self, architecture, config, driver, thermal, scenarios):
        self.bus_voltage = config.bus_voltage
        self.psu_family = Fixed.psu_family
        self.unit_rating = Fixed.psu_rating
        self.unit_price = Fixed.psu_price
        self.psu_url = Fixed.psu_url
        self.unit_count = 1
        self.psu_mass = Fixed.psu_mass_kg
        self.midpoint_price = Fixed.midpoint_balancer_price
        self.midpoint_mass = Fixed.midpoint_balancer_mass_kg
        self.zones = 1
        self.electronics_power = architecture.electronics_power + thermal.fan_power
        temperature_factor = thermal.hot_power_factor(Fixed.max_touch_temperature)
        self.event_trace = [(duration, power * temperature_factor + self.electronics_power)
                            for duration, power, _ in scenarios.reset_segments] * Inputs.events_back_to_back
        self.flight_trace = [(duration, power * temperature_factor + self.electronics_power)
                             for duration, power, _ in scenarios.flight_segments]
        self.event_peak_load = max(power for _, power in self.event_trace)
        self.peak_load = max(power for _, power in self.event_trace + self.flight_trace)
        self.sustained_load = thermal.sustained_power * temperature_factor + self.electronics_power
        self.required_rating = self.sustained_load * Fixed.psu_sizing_margin
        reset_deficit, depletion = self.trace_drawdown(self.event_trace)
        flight_deficit, _ = self.trace_drawdown(self.flight_trace)
        self.buffer_energy = max(reset_deficit, flight_deficit)
        self.burst_deficit_power = max(0.0, self.peak_load - self.unit_rating)
        self.buffer = EnergyBuffer(self.bus_voltage, self.zones, self.burst_deficit_power, self.buffer_energy)
        recharge_power = self.unit_rating - self.sustained_load
        self.recharge_window = Inputs.sustained_event_period - sum(duration for duration, _ in self.event_trace)
        self.recharge_time = depletion / recharge_power if recharge_power > 0 else float("inf")
        self.architecture_feasible = self.unit_rating >= self.required_rating and self.recharge_time <= self.recharge_window
        self.burst_rail_current = self.peak_load / (self.bus_voltage * (1 - Fixed.bus_droop_fraction))
        self.rail_imbalance_current = sqrt(driver.active_channels) * driver.current_offset_error

    def trace_drawdown(self, trace):
        depletion = 0.0
        peak_depletion = 0.0
        for duration, power in trace:
            depletion = max(0.0, depletion + (power - self.unit_rating) * duration)
            peak_depletion = max(peak_depletion, depletion)
        return peak_depletion, depletion

    def cells(self):
        return [
            Cell("Baseline PSU", self.psu_family),
            Cell("Baseline PSU rating (not reselected)", self.unit_rating, "W"),
            Cell("Sustained sizing requirement (hot bound)", self.required_rating, "W"),
            Cell("Peak across flight and reset (hot bound)", self.peak_load, "W"),
            Cell("Contact reset peak load (isolated-piece sum, hot bound)", self.event_peak_load, "W"),
            Cell("Always-on electronics / fans", self.electronics_power, "W"),
            Cell("Buffer energy requirement", self.buffer_energy, "J"),
            Cell("Buffer recharge time", self.recharge_time, "s"),
            Cell("Available recharge window", self.recharge_window, "s"),
            Cell("Peak rail current", self.burst_rail_current, "A"),
        ]


class StatusChecks:
    def __init__(self, board, coil, piece, snap, config, control, sensing, thermal, passive_thermal,
                 surface, drive, architecture, psu, scenarios, sim):
        authority = sim["verified_authority"]
        required_yaw_torque = piece.yaw_inertia * 4 * radians(Inputs.target_yaw_angle_deg) / Inputs.target_yaw_time ** 2
        tilt_margin = min(rung["tilt_torque"] / (piece.tilt_inertia * 4 * radians(rung["tilt_deg"]) / Inputs.target_tilt_time ** 2) for rung in authority["rungs"].values())
        self.results = {
            "Flight lift reserve": config.worst_available_margin >= Inputs.force_safety_factor,
            "Hot flight current screen": config.hot_sprint_current_margin >= 1,
            "Selected-coil coupled lift reserve": authority["lift_margin"] >= Inputs.force_safety_factor,
            "Full tilt-envelope lift reserve": authority["showpiece_lift_margin"] >= Inputs.force_safety_factor,
            "Coupled tilt manoeuvre authority": tilt_margin >= 1,
            "Coupled yaw manoeuvre authority": authority["yaw"] >= required_yaw_torque,
            "Parked-hover endurance screen": min(thermal.hover_endurance(scenarios.hover_worst_power), passive_thermal.hover_endurance(scenarios.hover_worst_power)) >= Inputs.min_level_hover_endurance,
            "Showpiece-hover endurance screen": min(thermal.hover_endurance(config.showpiece_hover_power), passive_thermal.hover_endurance(config.showpiece_hover_power)) >= Inputs.min_showpiece_hover_endurance,
            "Flight actuator rank": sim["actuator_rank6"] == 6,
            "Flight channel current": scenarios.required_flight_current <= config.current_limit,
            "Contact channel current, isolated piece": scenarios.required_reset_current <= config.current_limit,
            "Two-piece shared-coil reset probe": scenarios.contact_pair_current <= config.current_limit,
            "Powered-off neighbour snap screen": snap.snap_to_weight <= snap.holding_friction,
            "Composite throughput, fans on": scenarios.max_composite_moves_per_minute >= Inputs.sustained_moves_per_minute,
            "Composite throughput, fans off": scenarios.composite_duration <= Inputs.play_move_period,
            "Internal material temperature": max(thermal.peak_cell_temperature, passive_thermal.peak_cell_temperature) <= Inputs.coil_bed_temp_limit,
            "Every-cell brief-touch screen": max(thermal.peak_cell_temperature, passive_thermal.peak_cell_temperature) <= Fixed.max_touch_temperature,
            "Prolonged surface touch screen": surface.idle_touch_temperature <= Fixed.prolonged_touch_temperature,
            "Plate temperature": max(thermal.cyclic_peak_baseplate_temp, passive_thermal.cyclic_peak_baseplate_temp) <= Inputs.max_surface_temperature,
            "Visible hover height": surface.visible_hover >= Inputs.min_visible_hover_height,
            "Surface flatness": surface.flatness_fraction <= Inputs.max_surface_flatness_fraction,
            "Insulated radial winding fit": config.wire.radial_pitch * config.turns_per_layer + Fixed.coil_winding_clearance <= coil.winding_radial_width + Fixed.allocator_limit_tolerance,
            "Platform size": Inputs.platform_side_limits[0] <= board.platform_side <= Inputs.platform_side_limits[1],
            "Chess square size": board.square_size <= Fixed.max_chess_square_size,
            "Hall observability": sensing.worst_rank == 6,
            "Hall flight precision": sensing.flight_position_error_um <= sensing.required_flight_position_error_um,
            "Hall docking precision": sensing.total_position_error_um <= sensing.required_position_error_um,
            "Hall tilt precision": sensing.tilt_noise_mrad / 1000 * piece.box_height / 1000 <= Inputs.max_tip_position_error,
            "Hall flight saturation screen": sensing.saturation_field <= Fixed.hall_linear_range,
            "Hall unpowered-rest saturation screen": sensing.rest_saturation_field <= Fixed.hall_linear_range,
            "Hall throughput": sensing.headroom >= Fixed.min_hall_scan_headroom,
            "Ideal current slew screen": control.slew_time / 1000 * control.growth_rate <= Fixed.max_control_delay_fraction,
            "Control command rate screen": drive.current_command_rate_headroom >= 1,
            "Current bandwidth screen": drive.current_loop_headroom >= 1,
            "Current-regulator sampling ratio": drive.current_regulator_samples >= Fixed.min_current_loop_samples,
            "PWM ripple screen": control.current_ripple_peak <= Fixed.driver_max_ripple_current,
            "Switch peak-current screen": control.switch_current_bound <= Fixed.driver_peak_current,
            "Shunt dissipation": (max(scenarios.required_flight_current, scenarios.required_reset_current) ** 2 + control.current_ripple_peak ** 2) * Fixed.current_sense_resistance <= Fixed.shunt_power_derating * Fixed.current_shunt_power_rating,
            "Current offset": drive.driver.current_offset_error <= Fixed.max_current_offset_fraction * Fixed.driver_channel_current,
            "Current command resolution": drive.driver.setpoint_resolution <= Fixed.max_current_command_error,
            "Current monitoring": drive.architecture.current_monitor_headroom >= 1,
            "Serial command bandwidth": drive.driver.serial_headroom >= 1,
            "Compute capacity": architecture.compute_headroom >= Fixed.min_control_compute_headroom,
            "PSU and recharge": psu.architecture_feasible,
            "Bus current": psu.burst_rail_current <= Fixed.max_bus_current,
            "Midpoint balance screen": psu.rail_imbalance_current <= Fixed.midpoint_balancer_current_rating,
            "Supercap cell working voltage": Fixed.supercap_cell_working_voltage <= Fixed.supercap_cell_max_voltage,
            "Fan fit": thermal.fan_bank_fits,
        }
        self.unverified = (
            "Simultaneous reset shared-coil allocation and stationary-neighbour disturbance",
            "Collision-free reset from adversarial placements",
            "Physical board-edge actuation and sensing coverage",
            "Supplier-confirmed wire and winding process",
            "Delayed six-axis closed-loop stability across the motion envelope",
            "Measured acoustic and thermal acceptance",
            "Switching transients, cycle-average current sensing and bus/midpoint ripple",
        )

    @property
    def accepted(self):
        return all(self.results.values()) and not self.unverified

    def cells(self):
        return [Cell(name, "PASS" if passed else "FAIL") for name, passed in self.results.items()] + [
            Cell(name, "UNVERIFIED") for name in self.unverified]


class BomItem:
    def __init__(self, category, spec, qty_per_unit, unit_cost, link=""):
        self.category = category
        self.spec = spec
        self.qty_per_unit = qty_per_unit
        self.unit_cost = unit_cost
        self.link = link
        self.line_cost = qty_per_unit * unit_cost


class MassBudget:
    def __init__(self, board, wire, piece, thermal, psu):
        self.board_copper_mass = wire.copper_mass
        self.board_pcb_mass = board.motor_area * Fixed.pcb_thickness * Constants.fr4_density / 1000
        self.radiator_mass = thermal.aluminium_mass
        self.cooling_fan_mass = thermal.fan_mass
        self.buffer_mass = psu.buffer.mass
        self.gap_filler_mass = board.motor_area * Fixed.radiator_standoff_below_pcb / 1000 * Fixed.gap_filler_density / 1000
        bed_volume_l = board.motor_area * (thermal.coil_bed_thickness + Fixed.potting_thickness + Fixed.potting_cover_thickness) / 1e6
        self.potting_mass = (bed_volume_l - wire.copper_mass / (Constants.copper_density / 1000)) * Fixed.potting_density
        self.board_added_mass = (psu.psu_mass + psu.midpoint_mass + Fixed.frame_enclosure_mass_kg
                                 + Fixed.board_electronics_mass_kg + Fixed.bus_distribution_mass_kg)
        self.board_total_mass = (self.board_copper_mass + self.board_pcb_mass + self.radiator_mass + self.cooling_fan_mass
                                 + self.buffer_mass + self.gap_filler_mass + self.potting_mass + self.board_added_mass)
        self.piece_mass = piece.mass / 1000
        self.pieces_total = Inputs.piece_count
        self.all_pieces_mass = self.piece_mass * self.pieces_total
        self.set_total_mass = self.board_total_mass + self.all_pieces_mass

    def cells(self):
        return [
            Cell("Board copper (coils)", self.board_copper_mass, "kg"),
            Cell("Board PCB (FR4)", self.board_pcb_mass, "kg"),
            Cell("Aluminium baseplate + fins", self.radiator_mass, "kg"),
            Cell("Cooling fans", self.cooling_fan_mass, "kg"),
            Cell("Supercap burst buffer", self.buffer_mass, "kg"),
            Cell("Dispensed gap filler", self.gap_filler_mass, "kg"),
            Cell("Coil-bed potting epoxy", self.potting_mass, "kg"),
            Cell("PSU + midpoint + frame + electronics + cabling (est.)", self.board_added_mass, "kg"),
            Cell("Board total (est.)", self.board_total_mass, "kg"),
            Cell("Mass per piece", self.piece_mass * 1000, "g"),
            Cell("Pieces total", self.pieces_total),
            Cell("All pieces mass", self.all_pieces_mass, "kg"),
            Cell("WHOLE SET mass (est.)", self.set_total_mass, "kg"),
        ]


class BillOfMaterials:
    def __init__(self, board, coil, halbach, wire, config, architecture, sensing, thermal, psu, driver):
        power_mosfets = driver.half_bridges
        gate_drivers = driver.gate_drivers
        shift_registers = driver.shift_registers
        setpoint_filter_pairs = coil.total_bodies
        current_shunts = coil.total_bodies
        current_comparators = ceil(coil.total_bodies / Fixed.current_comparator_channels_per_ic)
        current_frontend_passives = coil.total_bodies * Fixed.current_frontend_passives_per_channel
        driver_passives = driver.half_bridges * Fixed.gate_passives_per_half_bridge
        driver_decoupling = shift_registers + gate_drivers + current_comparators
        hall_sensors = sensing.total_sensors
        hall_muxes = sensing.total_muxes
        hall_gate_switches = hall_muxes
        bulk_capacitors = architecture.bulk_capacitor_count
        thermal_sensors = architecture.thermal_sensor_count
        control_nodes = architecture.control_node_count
        setpoint_fpgas = architecture.setpoint_fpga_count
        motor_pcb_area_cm2 = board.motor_area / 100
        gap_filler_volume_cc = board.motor_area * Fixed.radiator_standoff_below_pcb / 1000
        assembly_joints = (
            power_mosfets * Fixed.power_mosfet_solder_joints
            + gate_drivers * Fixed.gate_driver_solder_joints
            + shift_registers * Fixed.shift_register_solder_joints
            + setpoint_filter_pairs * Fixed.setpoint_filter_solder_joints
            + current_shunts * Fixed.passive_solder_joints
            + current_comparators * Fixed.comparator_solder_joints
            + current_frontend_passives * Fixed.passive_solder_joints
            + driver_passives * Fixed.passive_solder_joints
            + driver_decoupling * Fixed.passive_solder_joints
            + bulk_capacitors * Fixed.passive_solder_joints
            + thermal_sensors * Fixed.passive_solder_joints
            + setpoint_fpgas * Fixed.board_setpoint_fpga_solder_joints
            + hall_sensors * Fixed.hall_sensor_solder_joints
            + hall_muxes * Fixed.hall_mux_solder_joints
            + hall_gate_switches * Fixed.gate_switch_solder_joints
            + control_nodes * Fixed.control_node_mcu_solder_joints
            + coil.total_bodies * Fixed.coil_solder_joints
        )

        self.piece_count = Inputs.piece_count
        self.motor_bed_items = [
            BomItem("Driver power MOSFET", ">=60V dual N-MOSFET SOP-8/PDFN, one half-bridge/package; RFQ target [TO BE SOURCED]", power_mosfets, Fixed.power_mosfet_price, ""),
            BomItem("Driver gate driver", "EG Micro EG2134, 3 half-bridges/IC (LCSC C480661)", gate_drivers, Fixed.gate_driver_price, "https://www.lcsc.com/product-detail/C480661.html"),
            BomItem("Current setpoint latch", f"GR74HC595 chains across {driver.setpoint_lane_count} parallel lanes", shift_registers, Fixed.shift_register_price, "https://www.lcsc.com/product-detail/C18164493.html"),
            BomItem("Setpoint RC filter", "15.8k 1% + 10nF X7R 0603 pair, 1.007kHz", setpoint_filter_pairs, Fixed.setpoint_filter_passive_price, "https://www.lcsc.com/product-detail/C519406.html"),
            BomItem("Board setpoint FPGA", f"board-wide delta-sigma engine, {driver.setpoint_lane_count} parallel outputs and >=55kbit channel state [TO BE SOURCED / SYNTHESIS REQUIRED]", setpoint_fpgas, Fixed.board_setpoint_fpga_price, ""),
            BomItem("Current shunt", "HoJLR2512-2W-20mR-1% midpoint-return sense", current_shunts, Fixed.current_shunt_price, "https://www.lcsc.com/product-detail/C2924538.html"),
            BomItem("Current comparator", "MSKSEMI LM393 dual comparator", current_comparators, Fixed.current_comparator_price, "https://www.lcsc.com/product-detail/C5252905.html"),
            BomItem("Current front-end passives", "0603 1% + matched-pair arrays; idle zero-calibrated [TO BE SOURCED]", current_frontend_passives, Fixed.current_frontend_passive_price, ""),
            BomItem("Local bulk capacitance", f"distributed per {Fixed.driver_channels_per_bulk_capacitor} drivers [TO BE SOURCED]", bulk_capacitors, Fixed.bulk_capacitor_price, ""),
            BomItem("Thermal sensor", "distributed 10k 1% NTCs for the per-cell energy observer [TO BE SOURCED]", thermal_sensors, Fixed.thermal_sensor_price, ""),
            BomItem("Driver gate passives", "0603 1% gate pull resistors", driver_passives, Fixed.driver_gate_passive_price, "https://www.lcsc.com/product-detail/C54531144.html"),
            BomItem("Driver decoupling", "100nF 50V X7R 0603 bypass capacitors", driver_decoupling, Fixed.driver_decoupling_price, "https://www.lcsc.com/product-detail/C14663.html"),
            BomItem("Motor-bed SMT assembly", "automated assembly joints over exact motor area", assembly_joints, Fixed.smt_assembly_cost_per_joint, "https://jlcpcb.com/help/article/pcb-assembly-faqs"),
            BomItem("Magnet wire", f"{config.wire.label} rectangular self-bonding enameled copper", wire.copper_mass, Fixed.wire_price_per_kg, "https://enameledwires.com/products/enameled-copper-wire/self-bonding-rectangular.html"),
            BomItem("Hall position sensor", "TI DRV5055A4QDBZR", hall_sensors, Fixed.hall_sensor_price, "https://www.digikey.com/en/products/detail/texas-instruments/DRV5055A4QDBZR/8567410"),
            BomItem("Hall group gate switch", "AO3401A -30V P-FET, one per mux group", hall_gate_switches, Fixed.hall_gate_switch_price, "https://www.lcsc.com/product-detail/MOSFETs_GOODWORK-AO3401A_C2938368.html"),
            BomItem("Hall readout mux", "TI CD74HC4067SM96 16-channel analog mux", hall_muxes, Fixed.hall_sensor_mux_price, "https://www.lcsc.com/product-detail/C98457.html"),
            BomItem("Motor/control PCB area", f"PCB area allowance over {board.motor_width:g}x{board.motor_height:g}mm motor", motor_pcb_area_cm2, Fixed.motor_pcb_price_per_cm2, "https://jlcpcb.com/news/discount-on-quality-4-layer-pcbs"),
            BomItem("Regional control MCU", "STM32G431KBT6-class node with two Hall scan engines", control_nodes, Fixed.control_node_mcu_price, "https://www.digikey.com/en/products/detail/stmicroelectronics/STM32G431KBT6/10231564"),
        ]
        self.piece_items = [
            BomItem("NdFeB magnet block", f"N48SH {Inputs.magnet_lateral_edge:g}x{Inputs.magnet_lateral_edge:g}x{Inputs.magnet_thickness:g}mm cube (SH grade) [TO BE SOURCED]", halbach.blocks_per_platform, halbach.block_mass / 1000 * Fixed.magnet_cost_per_kg, ""),
            BomItem("Piece plastic / misc", "PETG/PC print material plus inserts/finish allowance", 1, Fixed.piece_plastic_price, "https://jlc3dp.com/blog/3d-printing-cost"),
        ]
        self.board_items = [
            BomItem("Compute module", "RPi CM5 2GB Lite, SC1556", 1, Fixed.host_price, "https://www.digikey.com/en/products/detail/raspberry-pi/SC1556/25805567"),
            BomItem("Mainboard", "Custom carrier for CM5, power supervision and logical-board links", 1, Fixed.host_carrier_price, "https://jlcpcb.com/quote"),
            BomItem("Bus power supply", psu.psu_family, psu.unit_count, psu.unit_price, psu.psu_url),
            BomItem("Active bus midpoint", f"controlled {psu.bus_voltage:g}V-to-+/-{psu.bus_voltage / 2:g}V midpoint balance and protection [TO BE SOURCED / BENCH VALIDATION REQUIRED]", 1, psu.midpoint_price, ""),
            BomItem("Bus distribution", "Copper 110 flat busbar plus zone cabling allowance", 1, Fixed.bus_distribution_price, "https://www.ebay.com/itm/304578689563"),
            BomItem("Rail regen clamp", "TVS + dump resistor per rail per zone; PSUs cannot sink braking energy [TO BE SOURCED]", 2 * psu.zones, Fixed.rail_clamp_price, ""),
            BomItem("Rail bulk capacitance", "Low-ESR bulk electrolytic per rail per zone [TO BE SOURCED]", 2 * psu.zones, Fixed.rail_bulk_capacitor_price, ""),
            BomItem("Supercap burst buffer", f"Maxwell BCAP0350-P270-S18, {psu.buffer.series_cells}s{psu.buffer.strings_per_rail}p per rail", psu.buffer.cell_count, Fixed.supercap_cell_price, "https://www.digikey.com/en/products/detail/maxwell-technologies/BCAP0350-P270-S18/11673891"),
            BomItem("Supercap balancing", "active balancing network per cell group [TO BE SOURCED]", psu.buffer.cell_count, Fixed.supercap_balancer_price_per_cell, ""),
            BomItem("Buffer charge/protection", f"{psu.buffer.series_cells}-series monitor + precharge/charge + fuse/disconnect [TO BE SOURCED]", psu.buffer.management_count, Fixed.supercap_management_price_per_rail, ""),
            BomItem("Buffer charge/discharge protection", f"protected high-current paths around both rail banks; {psu.burst_rail_current:.0f}A modeled rail demand [TO BE SOURCED]", psu.buffer.oring_count, Fixed.supercap_oring_price, ""),
            BomItem("Radiator aluminium", "Integral-fin extrusion, crosshatch-kerfed 4mm base", thermal.aluminium_mass, Fixed.radiator_aluminium_price_per_kg, ""),
            BomItem("Radiator eddy-break slotting", "gang-saw/CNC 5mm-pitch crosshatch", 1, Fixed.radiator_slotting_price, ""),
            BomItem("Coil potting epoxy", "2.5 W/mK filled epoxy allowance", 1, Fixed.potting_epoxy_price, "https://www.ziitek.com/epoxy-potting-compound"),
            BomItem("Dispensed thermal gap filler", f"5.6 W/mK, {Fixed.radiator_standoff_below_pcb}mm bond line, {gap_filler_volume_cc:.1f}cc", 1, Fixed.gap_filler_pad_price, "https://www.laird.com/products/thermal-interface-materials/liquid-gap-fillers/tputty-sf560"),
            BomItem("Playing surface", "UV-printed graphics + clear wear topcoat [TO BE SOURCED]", 1, Fixed.playing_surface_price, ""),
            BomItem("AC input", "IEC inlet + fuse + mains switch + harness [TO BE SOURCED]", 1, Fixed.ac_input_price, ""),
            BomItem("Mains EMI filter", "conducted-emissions filter [TO BE SOURCED]", 1, Fixed.emi_filter_price, ""),
            BomItem("Enclosure / frame", "frame + skirt allowance [TO BE SOURCED]", 1, Fixed.enclosure_price, ""),
        ]
        if thermal.fan_count:
            self.board_items.append(BomItem("Radiator fan", "Noctua NF-A20 PWM 200mm at 550rpm", thermal.fan_count, Fixed.cooling_fan_price, Fixed.cooling_fan_url))

        self.board_items = [item for item in self.board_items if item.qty_per_unit]
        self.motor_bed_cost = sum(i.line_cost for i in self.motor_bed_items)
        self.per_piece_cost = sum(i.line_cost for i in self.piece_items)
        self.board_shared_cost = sum(i.line_cost for i in self.board_items)
        self.pieces_cost = self.per_piece_cost * self.piece_count
        self.total = self.motor_bed_cost + self.pieces_cost + self.board_shared_cost


def format_value(value):
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    if value != 0 and (abs(value) < Fixed.report_scientific_lower_bound or abs(value) >= Fixed.report_scientific_upper_bound):
        return f"{value:.{Fixed.report_precision}g}"
    text = f"{value:.{Fixed.report_precision}f}".rstrip("0").rstrip(".")
    return text


def print_section(title, cells):
    print()
    print(title)
    print("-" * len(title))
    for cell in cells:
        suffix = f" {cell.unit}" if cell.unit else ""
        print(f"  {cell.label:<{Fixed.report_label_width}} {format_value(cell.value)}{suffix}")


def print_sweep(sweep):
    print_section("Winding screen (flight only; hardware remains a baseline)", [
        Cell(f"{candidate.bus_voltage:g} V: {candidate.wire.label}, {candidate.turns} turns", candidate.worst_sprint_piece_power, "W peak-current-allocation copper estimate")
        for candidate in sweep.best_per_voltage])


def print_bom_group(items):
    for item in items:
        link = f"  {item.link}" if item.link else ""
        print(f"  {item.category:<21}{item.spec:<46}qty {format_value(item.qty_per_unit):>8}  ${item.unit_cost:>7.3f}  ${item.line_cost:>9.2f}{link}")


def print_bom(bill):
    print()
    title = f"BOM (per board; cost assumptions for {Inputs.production_boards}-board order, sourcing unverified)"
    print(title)
    print("-" * len(title))

    print()
    print("  [A] Continuous motor/control bed (one logical board)")
    print_bom_group(bill.motor_bed_items)
    print(f"  {'MOTOR-BED SUBTOTAL':<21}{'':<46}{'':>12}  {'':>8}  ${bill.motor_bed_cost:>9.2f}")

    print()
    print("  [B] Per piece (chess piece)")
    print_bom_group(bill.piece_items)
    print(f"  {'PER-PIECE SUBTOTAL':<21}{'':<46}{'':>12}  {'':>8}  ${bill.per_piece_cost:>9.2f}")

    print()
    print("  [C] Board-shared (one per board)")
    print_bom_group(bill.board_items)
    print(f"  {'SHARED SUBTOTAL':<21}{'':<46}{'':>12}  {'':>8}  ${bill.board_shared_cost:>9.2f}")

    print()
    print("  Board roll-up")
    print(f"  {'Motor/control bed':<21}{'one logical board':<46}{'':>12}  {'':>8}  ${bill.motor_bed_cost:>9.2f}")
    print(f"  {'Pieces':<21}{f'{bill.piece_count} x ${bill.per_piece_cost:.2f}':<46}{'':>12}  {'':>8}  ${bill.pieces_cost:>9.2f}")
    print(f"  {'Board-shared':<21}{'':<46}{'':>12}  {'':>8}  ${bill.board_shared_cost:>9.2f}")
    print(f"  {'BOARD TOTAL':<21}{'':<46}{'':>12}  {'':>8}  ${bill.total:>9.2f}")


def print_report(result):
    print("LEVITATING CHESS: SINGLE-PIECE FLIGHT / CONTACT RESET")
    print("ACCEPTED" if result["checks"].accepted else "NOT ACCEPTED: failed or unverified requirements remain")
    for title, cells in result["sections"]:
        print_section(title, cells)
    if result["sweep"] is not None:
        print_sweep(result["sweep"])
    print_bom(result["bom"])


def run_report(output_path, include_winding_search=False, include_mode_analysis=False):
    result = calculate_model(include_mode_analysis=include_mode_analysis, include_winding_search=include_winding_search)
    output = StringIO()
    with redirect_stdout(output):
        print_report(result)
    report = output.getvalue()
    Path(output_path).write_text(report)
    print(report, end="")
    return result


class MotionScenarios:
    def __init__(self, config, allocations, control):
        self.control = control
        self.hover_worst_power = self.losses(config, allocations["hover"])
        flight_worst = self.losses(config, allocations["flight"])
        reset_worst = self.losses(config, allocations["reset"])
        breakaway_worst = self.losses(config, allocations["breakaway"])
        self.flight_segments = [
            (Inputs.worst_phase_dwell, self.hover_worst_power, self.hover_worst_power),
            (Inputs.cruise_duration, flight_worst, flight_worst),
            (Inputs.landing_dwell, self.hover_worst_power, self.hover_worst_power),
        ]
        breakaway_duration = Inputs.reset_breakaway_duration
        if not 0 < breakaway_duration <= Inputs.reset_duration:
            raise ValueError("Reset breakaway duration must fit the reset")
        self.reset_segments = [(breakaway_duration, breakaway_worst * Inputs.piece_count, breakaway_worst),
                               (Inputs.reset_duration - breakaway_duration, reset_worst * Inputs.piece_count, reset_worst)]
        self.reset_segments = [segment for segment in self.reset_segments if segment[0] > 0]
        self.flight_energy = sum(duration * power for duration, power, _ in self.flight_segments)
        self.reset_energy = sum(duration * power for duration, power, _ in self.reset_segments)
        self.flight_duration = sum(duration for duration, _, _ in self.flight_segments)
        self.moves_per_composite = 1 + Inputs.replay_capture_fraction + Inputs.replay_knight_gap_fraction * Fixed.blocker_hops_per_gap_move
        self.composite_duration = self.flight_duration * self.moves_per_composite
        self.composite_energy = self.flight_energy * self.moves_per_composite
        self.max_composite_moves_per_minute = 60 / self.composite_duration
        self.contact_tipping_margin = allocations["contact_tipping_margin"]
        self.contact_pair_current = allocations["contact_pair"]["peak_ampere_turns"] / config.turns
        self.required_flight_current = allocations["flight"]["peak_ampere_turns"] / config.turns
        self.required_reset_current = max(allocations[name]["peak_ampere_turns"] for name in ("reset", "breakaway")) / config.turns

    def losses(self, config, data):
        squared_sum = data["worst_squared_sum"] / config.turns ** 2 + data["coil_count"] * self.control.current_ripple_peak ** 2
        absolute_sum = data["worst_absolute_sum"] / config.turns + data["coil_count"] * self.control.current_ripple_peak
        conduction = squared_sum * (config.resistance + Fixed.driver_hot_resistance + Fixed.current_sense_resistance)
        switching = config.bus_voltage * absolute_sum * Fixed.driver_switching_time * Fixed.driver_pwm_frequency
        gates = data["coil_count"] * Fixed.driver_pwm_frequency * 2 * Fixed.driver_mosfet_gate_charge * Fixed.gate_drive_voltage
        return conduction + switching + gates

    def cells(self):
        return [
            Cell("Flight concurrency", Inputs.pieces_levitating_simultaneously),
            Cell("Contact-reset participants", Inputs.piece_count),
            Cell("Minimum flight channel rating (nominal-temperature sampled poses)", self.required_flight_current, "A"),
            Cell("Minimum contact channel rating (nominal temperature, isolated piece)", self.required_reset_current, "A"),
            Cell("Two touching pieces, opposing reset commands", self.contact_pair_current, "A/channel"),
            Cell("Passive friction tipping margin (information)", self.contact_tipping_margin, "x"),
            Cell("Flight energy at required current (hardware acceptance separate)", self.flight_energy, "J"),
            Cell("Reset energy (isolated-piece sum)", self.reset_energy, "J"),
            Cell("Composite serial movement duration", self.composite_duration, "s"),
            Cell("Maximum composite throughput at current timings", self.max_composite_moves_per_minute, "moves/min"),
            Cell("Required composite throughput", Inputs.sustained_moves_per_minute, "moves/min"),
        ]


def select_hall_pitch(board, coil, control, driver, config, piece):
    for pitch in Fixed.hall_pitch_candidates:
        architecture = BoardControl(board, coil, control, driver, pitch)
        sensing = HallSensing(coil, config, control, architecture)
        if (sensing.worst_rank == 6 and sensing.worst_condition <= 2 ** Fixed.hall_interpolation_bits
                and sensing.total_position_error_um <= sensing.required_position_error_um
                and sensing.flight_position_error_um <= sensing.required_flight_position_error_um
                and sensing.saturation_field <= Fixed.hall_linear_range
                and sensing.headroom >= Fixed.min_hall_scan_headroom
                and sensing.tilt_noise_mrad / 1000 * piece.box_height / 1000 <= Inputs.max_tip_position_error):
            return architecture, sensing
    raise ValueError("No Hall pitch satisfies sensing requirements")


def calculate_model(include_mode_analysis=False, include_winding_search=False):
    board = BoardGeometry()
    coil = CoilBed(board)
    halbach = HalbachArray(board)
    piece = Piece(board, halbach)
    geometry = levitation_sim.SimGeometry(Inputs, Fixed, Constants, board, coil, piece)
    sim = levitation_sim.measure(geometry)
    snap = NeighbourSnap(board, piece, halbach, sim)
    sweep = ConfigurationSweep(coil, piece, sim) if include_winding_search else None
    config = selected_winding(coil, piece, sim)
    coil.outer_height = config.coil_height
    sim["verified_authority"] = levitation_sim.verified_worst_authority(coil.control_cells_per_side, config.coil_height / 1000, config.turns * config.current_limit)
    allocations = levitation_sim.operating_allocations(coil.control_cells_per_side, config.coil_height / 1000, config.turns * config.current_limit)
    driver = DiscreteDriver(coil)
    control = Control(coil, config, allocations)
    scenarios = MotionScenarios(config, allocations, control)
    architecture, sensing = select_hall_pitch(board, coil, control, driver, config, piece)
    drive = DriveMatrix(control, driver, architecture)
    thermal = RadiatorCooling(board, config, Inputs.active_cooling_fans, True, scenarios, architecture)
    passive_thermal = RadiatorCooling(board, config, 0, False, scenarios, architecture)
    surface = SurfaceStack(thermal)
    psu = PowerSupply(architecture, config, driver, thermal, scenarios)
    wire = WindingInventory(coil, config)
    bom = BillOfMaterials(board, coil, halbach, wire, config, architecture, sensing, thermal, psu, driver)
    mass = MassBudget(board, wire, piece, thermal, psu)
    checks = StatusChecks(board, coil, piece, snap, config, control, sensing, thermal, passive_thermal,
                          surface, drive, architecture, psu, scenarios, sim)
    sections = [("Board geometry", board), ("Coil geometry", coil), ("Magnet array", halbach),
                ("Piece", piece), ("Neighbour snap", snap), ("Selected winding screen", config),
                ("Motion scenarios", scenarios), ("Control screen", control), ("Drivers", driver),
                ("Control architecture", architecture), ("Hall sensing", sensing), ("Drive timing", drive),
                ("Fan-assisted thermal screen", thermal), ("Passive thermal screen", passive_thermal),
                ("Surface", surface), ("Supply", psu), ("Optional energy buffer", psu.buffer),
                ("Mass", mass), ("Acceptance status", checks)]
    sections = [(title, component.cells()) for title, component in sections]
    sections.insert(0, ("Magnetic diagnostics", [Cell("Peak reference-plane Bz", sim["peak_bz"], "T"),
                                               Cell("Worst sampled normalized wrench condition", sim["actuator_condition6"])]))
    if include_mode_analysis:
        axial = min(Fixed.max_coil_axial_filaments, max(Fixed.minimum_axial_filaments, ceil(config.coil_height / 1000 / Fixed.axial_filament_spacing)))
        poses = levitation_sim.flight_worst_case(coil.control_cells_per_side, piece.weight, board.platform_side / 2000,
                                                 (geometry.gap, geometry.max_flight_gap), config.coil_height / 1000, axial)
        modes = levitation_sim.local_current_mode_analysis(poses["wrenches"], poses["level_all_wrenches"], piece.weight,
                                                           board.platform_side / 2000, piece.mass / 1000 * geometry.cruise_accels[0])
        sections.append(("Optional current-mode diagnostic (no driver-count credit)", [
            Cell(f"{row['basis_source']} / {row['modes']} modes: peak current ratio", row["worst_peak_current_ratio"], "x")
            for row in modes]))
    return {"sections": sections, "sweep": sweep, "bom": bom, "checks": checks}


def main():
    parser = argparse.ArgumentParser(description="Evaluate the adopted design; optional searches do not change its settings.")
    parser.add_argument('--winding-search', action='store_true', help="Include candidate winding comparisons")
    parser.add_argument('--mode-analysis', action='store_true', help="Include the current-mode compression diagnostic")
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent.parent / 'last_run.txt')
    arguments = parser.parse_args()
    run_report(arguments.output, include_winding_search=arguments.winding_search, include_mode_analysis=arguments.mode_analysis)


if __name__ == "__main__":
    main()
