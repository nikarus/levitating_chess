from contextlib import redirect_stdout
from io import StringIO
from math import pi, sqrt, exp, ceil, floor, sin, radians, log10
from pathlib import Path

import levitation_sim


class Inputs:
    magnet_cube_edge = 5                       # mm
    magnets_per_period = 4                     # count
    periods_per_side = 1                       # count
    coils_per_period = 4                       # count
    control_cells_per_side = 8                 # count
    magnet_to_coil_distance = 3                # mm
    plastic_wall_thickness = 1.0               # mm
    max_hover_duration = 2                     # s
    spot_cooldown_duration = 60                # s
    allowed_wire_temp_rise = 40                # K
    force_safety_factor = 1.3                  # ratio
    min_maneuver_accel_g = 0.2                 # g
    target_tilt_angle_deg = 10                 # deg
    target_tilt_time = 0.3                     # s
    target_yaw_angle_deg = 90                  # deg
    target_yaw_time = 0.5                      # s
    position_sense_resolution_um = 5           # um
    ambient_temperature = 35                   # C
    max_surface_temperature = 50               # C
    control_loop_bandwidth_margin = 5          # ratio
    pieces_levitating_simultaneously = 32      # count
    drive_look_ahead_factor = 1.5              # ratio
    production_volume = 100                    # boards
    active_cooling_fans = 3                    # count


class Fixed:
    base_corner_standoff = 8                   # mm
    square_fill_ratio = 0.8                    # ratio
    resting_friction_coefficient = 0.4         # ratio
    captured_pieces_total = 32                 # count
    captured_side_areas = 2                    # count
    captured_board_gap = 10                    # mm
    herringbone_orientation_families = 2       # count
    com_height_fraction = 0.4                  # ratio
    reference_king_height = 95                 # mm
    reference_king_base_diameter = 44          # mm
    rectangular_wire_film = 0.012              # mm per side
    coil_aspect_ratio_target = 2.5             # ratio
    turns_per_radial_layer = 1                 # count
    winding_radial_width_factor = 0.35         # ratio
    nominal_coil_height_for_field = 1.0        # mm
    potting_thickness = 1.0                    # mm
    potting_thermal_conductivity = 1.0         # W/(m.K)
    pcb_via_effective_thermal_conductivity = 5 # W/(m.K)
    thermal_pad_thickness = 0.5                # mm
    thermal_pad_conductivity = 5.0             # W/(m.K)
    baseplate_thickness = 4.0                  # mm
    aluminium_thermal_conductivity = 167       # W/(m.K)
    aluminium_density = 2700                   # kg/m3
    aluminium_heat_capacity = 900              # J/(kg.K)
    fin_height = 15                            # mm
    fin_thickness = 2                          # mm
    fin_channel_width = 8                      # mm
    natural_convection_coefficient = 3         # W/(m2.K)
    forced_convection_coefficient = 8          # W/(m2.K)
    cooling_fan_size = 200                     # mm
    cooling_fan_speed = 550                    # rpm
    cooling_fan_airflow = 100.8                # m3/h
    cooling_fan_static_pressure = 0.51         # mm H2O
    cooling_fan_noise = 10.7                   # dB(A)
    cooling_fan_installation_noise = 3         # dB(A)
    cooling_fan_airflow_fraction = 0.25        # ratio
    cooling_fan_power = 0.96                   # W
    cooling_fan_mass = 0.370                   # kg
    cooling_fan_price = 39.95                  # USD
    cooling_fan_url = "https://www.coolerguys.com/products/noctua-nf-a20-pwm-200mm-cooling-fan"
    mosfet_voltage_rating = 60                 # V
    driver_channel_current = 2.5               # A
    driver_pwm_frequency = 20000               # Hz
    driver_switching_time = 100e-9             # s
    driver_hot_resistance = 0.08               # ohm
    driver_mosfet_gate_charge = 30e-9          # C
    gate_drive_voltage = 10                    # V
    logic_gate_voltage = 5                     # V
    current_command_bits_per_channel = 12      # count
    current_command_rate = 500                 # Hz
    current_pwm_resolution_bits = 10           # count
    current_setpoint_dac_passives_per_channel = current_command_bits_per_channel + 1
    max_current_command_error = 0.005          # A
    current_loop_bandwidth = 2000              # Hz
    min_current_loop_pwm_cycles = 8            # count
    current_monitor_adc_sample_rate = 500000   # samples/s
    current_sense_resistance = 0.02            # ohm
    current_sense_common_mode_voltage = 26     # V
    current_shunt_price = 0.0343               # USD
    current_sense_amp_price = 0.1327           # USD
    current_comparator_channels_per_ic = 2     # count
    current_comparator_price = 0.0198          # USD
    current_frontend_passives_per_channel = 2  # count
    current_frontend_passive_price = 0.0010    # USD
    shift_register_outputs = 8                 # count
    gate_driver_half_bridges = 3               # count
    gate_driver_price = 0.2254                 # USD
    power_mosfet_price = 0.0696                # USD
    shift_register_clock_rating = 25000000     # bits/s
    shift_register_power_capacitance = 42e-12   # F
    shift_register_price = 0.0280              # USD
    driver_serial_clock = 40000000             # bits/s
    smt_assembly_cost_per_joint = 0.0017       # USD
    max_bus_current = 80                       # A
    usable_bus_voltage_fraction = 0.9          # ratio
    hall_sensor_pitch = 6.667                  # mm
    hall_observation_window_side = 4           # count
    hall_sensor_mux_channels = 16              # count
    hall_sensor_price = 0.2067                 # USD
    hall_sensor_mux_price = 0.2527             # USD
    hall_adc_sample_rate = 500000              # samples/s
    hall_interpolation_bits = 12               # bits
    windings_per_coil_body = 1                 # count
    pcb_thickness = 1.6                        # mm
    psu_mass_kg = 0.62                         # kg
    frame_enclosure_mass_kg = 1.0              # kg
    board_electronics_mass_kg = 0.3            # kg
    control_tile_side = 100                    # mm
    piece_control_flops = 20000                # flop/update
    node_mcu_throughput_mflops = 170           # Mflop/s
    tile_mcu_power = 0.4                       # W
    host_power = 8                             # W
    psu_sizing_margin = 1.25                   # ratio
    psu_options = {
        24: ("4x Mean Well UHP-500-24, 24V 2006W fanless PSU bank", 2006.4, 376.80, "https://www.digikey.com/en/products/detail/mean-well-usa-inc/UHP-500-24/8324036"),
    }


class Constants:
    board_squares_per_side = 8
    ndfeb_remanence_br = 1.45
    ndfeb_density = 0.0075
    plastic_density = 0.0012
    copper_resistivity = 1.724e-08
    copper_density = 8960
    winding_conductor_thicknesses = [0.05, 0.07, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5]
    standard_bus_voltages = [24]
    gravity = 9.80665
    vacuum_permeability = 1.25663706e-6
    fr4_density = 0.00185


def interpolate_height_coupling(sim, coil_height_mm):
    heights = sim["coil_height_coupling_heights_mm"]
    factors = sim["coil_height_coupling_factors"]
    if coil_height_mm <= heights[0]:
        return factors[0]
    if coil_height_mm >= heights[-1]:
        return factors[-1]
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
        self.period_length = Inputs.magnets_per_period * Inputs.magnet_cube_edge
        self.platform_side = Inputs.periods_per_side * self.period_length
        self.base_diameter = self.platform_side * sqrt(2) + 2 * Fixed.base_corner_standoff
        self.square_size = self.base_diameter / Fixed.square_fill_ratio
        self.board_side = Constants.board_squares_per_side * self.square_size
        self.captured_per_side = ceil(Fixed.captured_pieces_total / Fixed.captured_side_areas)
        self.captured_rows = max(1, floor(self.board_side / self.base_diameter))
        self.captured_columns = ceil(self.captured_per_side / self.captured_rows)
        self.storage_width_each = Fixed.captured_board_gap + self.captured_columns * self.base_diameter
        self.motor_width = self.board_side + Fixed.captured_side_areas * self.storage_width_each
        self.motor_height = self.board_side
        self.motor_area = self.motor_width * self.motor_height

    def cells(self):
        return [
            Cell("Magnetic period length", self.period_length, "mm"),
            Cell("Platform square side", self.platform_side, "mm"),
            Cell("Round base diameter", self.base_diameter, "mm"),
            Cell("Chess square size", self.square_size, "mm"),
            Cell("Active board side", self.board_side, "mm"),
            Cell("Captured-zone rows (touching)", self.captured_rows),
            Cell("Captured-zone columns (touching)", self.captured_columns),
            Cell("Captured-zone width each side", self.storage_width_each, "mm"),
            Cell("Total motor width", self.motor_width, "mm"),
            Cell("Total motor height", self.motor_height, "mm"),
            Cell("Total motor area", self.motor_area, "mm2"),
        ]


class CoilBed:
    def __init__(self, board):
        self.outer_width = board.period_length / Inputs.coils_per_period
        self.columns = Inputs.coils_per_period * Inputs.periods_per_side
        self.rows = max(1, round(board.platform_side / (Fixed.coil_aspect_ratio_target * self.outer_width)))
        self.outer_length = board.platform_side / self.rows
        self.outer_height = 0.0
        self.aspect_ratio = self.outer_length / self.outer_width
        self.bodies_per_orientation = self.columns * self.rows
        self.bodies_under_platform = Fixed.herringbone_orientation_families * self.bodies_per_orientation
        self.control_cells_per_side = Inputs.control_cells_per_side
        self.control_columns = self.control_cells_per_side
        self.control_rows = max(1, round(self.control_cells_per_side * self.outer_width / self.outer_length))
        self.control_bed_per_orientation = self.control_columns * self.control_rows
        self.control_bed_bodies = Fixed.herringbone_orientation_families * self.control_bed_per_orientation
        self.footprint_area = self.outer_width * self.outer_length
        self.winding_radial_width = self.outer_width * Fixed.winding_radial_width_factor
        self.conductor_radial_width = self.winding_radial_width / Fixed.turns_per_radial_layer
        self.body_density = self.bodies_under_platform / (board.platform_side ** 2)
        self.coil_spacing = sqrt(1 / self.body_density)
        self.total_bodies = ceil(board.motor_area * self.body_density)
        self.windings = self.total_bodies * Fixed.windings_per_coil_body
        self.active_bodies = self.control_bed_bodies * Inputs.pieces_levitating_simultaneously
        self.active_windings = self.active_bodies * Fixed.windings_per_coil_body
        self.peak_driven_windings = ceil(self.active_windings * Inputs.drive_look_ahead_factor)
        self.half_bridges_per_coil = 2
        self.drive_voltage_fraction = 1.0

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
            Cell("Active bodies (all pieces at once)", self.active_bodies),
            Cell("Lift windings (pieces at once)", self.active_windings),
            Cell("Peak driven windings (+thrust look-ahead)", self.peak_driven_windings),
            Cell("Dedicated coil driver channels", self.total_bodies),
        ]


class HalbachArray:
    def __init__(self, board, sim):
        self.blocks_per_side = Inputs.periods_per_side * Inputs.magnets_per_period
        self.block_volume = Inputs.magnet_cube_edge ** 3
        self.blocks_per_platform = self.blocks_per_side ** 2
        self.block_mass = self.block_volume * Constants.ndfeb_density
        self.magnet_mass = self.blocks_per_platform * self.block_mass
        self.circumradius = board.platform_side / 2 * sqrt(2)
        self.resting_magnet_gap = board.base_diameter - 2 * self.circumradius
        self.b_at_coils = sim["peak_bz"]

    def cells(self):
        return [
            Cell("Magnet blocks per side", self.blocks_per_side),
            Cell("Magnet block volume", self.block_volume, "mm3"),
            Cell("Magnet blocks per platform", self.blocks_per_platform),
            Cell("Magnet block mass", self.block_mass, "g"),
            Cell("Magnet mass total", self.magnet_mass, "g"),
            Cell("Magnet corner reach (circumradius)", self.circumradius, "mm"),
            Cell("Resting magnet gap (bases touching)", self.resting_magnet_gap, "mm"),
            Cell("Simulated peak Bz at coil plane", self.b_at_coils, "T"),
        ]


class Piece:
    def plastic_shell_volume(self, diameter, height, wall_thickness):
        return pi / 4 * (diameter ** 2 * height - (diameter - 2 * wall_thickness) ** 2 * (height - 2 * wall_thickness))

    def __init__(self, board, halbach):
        self.scale = board.base_diameter / Fixed.reference_king_base_diameter
        self.box_height = Fixed.reference_king_height * self.scale
        self.diameter = board.base_diameter
        self.shell_volume = self.plastic_shell_volume(self.diameter, self.box_height, Inputs.plastic_wall_thickness)
        self.shell_mass = self.shell_volume * Constants.plastic_density
        self.mass = self.shell_mass + halbach.magnet_mass
        self.magnet_fraction = halbach.magnet_mass / self.mass
        self.weight = self.mass / 1000 * Constants.gravity

    def cells(self):
        return [
            Cell("Size scale", self.scale),
            Cell("Box height (scaled king)", self.box_height, "mm"),
            Cell("Cylinder diameter", self.diameter, "mm"),
            Cell("Plastic shell volume", self.shell_volume, "mm3"),
            Cell("Plastic shell mass", self.shell_mass, "g"),
            Cell("Piece mass (real)", self.mass, "g"),
            Cell("Magnet mass fraction", self.magnet_fraction),
            Cell("Piece weight", self.weight, "N"),
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


class CoilConfiguration:
    def __init__(self, coil, halbach, piece, sim, conductor_thickness, bus_voltage, layers):
        self.wire = rectangular_wire(coil.conductor_radial_width, conductor_thickness)
        self.bus_voltage = bus_voltage
        self.turns_per_layer = Fixed.turns_per_radial_layer
        self.radial_width = coil.winding_radial_width
        self.layers = layers
        self.turns = self.turns_per_layer * self.layers
        self.inner_window_width = coil.outer_width - 2 * self.radial_width
        self.inner_window_length = coil.outer_length - 2 * self.radial_width
        self.coil_height = self.layers * self.wire.axial_pitch
        self.height_coupling = interpolate_height_coupling(sim, self.coil_height)
        self.average_length_per_turn = 2 * ((coil.outer_length - self.radial_width) + (coil.outer_width - self.radial_width))
        self.length_per_winding = self.turns * self.average_length_per_turn / 1000
        self.cross_section_area = self.wire.copper_area
        self.resistance = Constants.copper_resistivity * self.length_per_winding / self.cross_section_area
        self.usable_drive_voltage = bus_voltage * coil.drive_voltage_fraction * Fixed.usable_bus_voltage_fraction
        self.voltage_limited_current = self.usable_drive_voltage / self.resistance
        self.current_limit = min(self.voltage_limited_current, Fixed.driver_channel_current)
        self.force_per_amp = self.turns * sim["lift_force_per_ampere_turn"] * self.height_coupling
        self.available_force = self.current_limit * self.force_per_amp
        self.available_margin = self.available_force / piece.weight
        self.required_current = piece.weight * Inputs.force_safety_factor / self.force_per_amp
        self.operating_current = self.required_current
        self.hover_current = self.required_current / Inputs.force_safety_factor
        self.piece_hover_power = self.resistance * sim["hover_ampere_turns_squared_sum"] / self.turns ** 2 / self.height_coupling ** 2
        self.worst_force_per_amp = self.turns * sim["worst_lift_force_per_ampere_turn"] * self.height_coupling
        self.worst_available_force = self.current_limit * self.worst_force_per_amp
        self.worst_available_margin = self.worst_available_force / piece.weight
        self.worst_required_current = piece.weight * Inputs.force_safety_factor / self.worst_force_per_amp
        self.worst_piece_hover_power = self.resistance * sim["worst_hover_ampere_turns_squared_sum"] / self.turns ** 2 / self.height_coupling ** 2
        self.worst_case_poses = sim["worst_case_poses"]
        self.worst_case_max_tilt_deg = sim["worst_case_max_tilt_deg"]
        self.total_force = self.operating_current * self.force_per_amp
        self.force_per_body = self.total_force / coil.bodies_under_platform
        self.margin = self.total_force / piece.weight
        self.voltage_per_winding = self.operating_current * self.resistance
        self.power_per_winding = self.operating_current ** 2 * self.resistance

    def cells(self):
        return [
            Cell("Selected wire", self.wire.label),
            Cell("Winding radial pitch", self.wire.radial_pitch, "mm"),
            Cell("Winding axial pitch", self.wire.axial_pitch, "mm"),
            Cell("Copper cross-section area", self.cross_section_area * 1000000, "mm2"),
            Cell("Selected bus voltage", self.bus_voltage, "V"),
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
            Cell("Worst-case max tilt swept", self.worst_case_max_tilt_deg, "deg"),
            Cell("Worst-case lift force per amp (sim)", self.worst_force_per_amp, "N/A"),
            Cell("Worst-case required lift current", self.worst_required_current, "A"),
            Cell("Worst-case available lift margin", self.worst_available_margin, "x"),
            Cell("Worst-case hover power (one piece)", self.worst_piece_hover_power, "W"),
            Cell("Operating current per winding", self.operating_current, "A"),
            Cell("Lift force per coil body", self.force_per_body, "N"),
            Cell("Total lift force", self.total_force, "N"),
            Cell("Lift margin (with safety)", self.margin, "x"),
        ]


class ConfigurationSweep:
    def copper_proxy(self, c):
        return c.length_per_winding * c.cross_section_area

    def is_feasible(self, board, coil, halbach, piece, sim, c):
        if c.inner_window_width <= 0 or c.inner_window_length <= 0:
            return False
        if c.available_margin < Inputs.force_safety_factor:
            return False
        if c.bus_voltage > Fixed.mosfet_voltage_rating:
            return False
        if c.bus_voltage > Fixed.current_sense_common_mode_voltage:
            return False
        driver = DiscreteDriver(board, coil, c)
        cooling = RadiatorCooling(board, c, driver, Inputs.active_cooling_fans)
        if cooling.cyclic_peak_baseplate_temp > Inputs.max_surface_temperature:
            return False
        if cooling.cyclic_peak_source_temp > Inputs.ambient_temperature + Inputs.allowed_wire_temp_rise:
            return False
        prop = Propulsion(board, coil, piece, c, halbach, sim)
        if prop.acceleration_in_g < Inputs.min_maneuver_accel_g:
            return False
        att = AttitudeAuthority(board, piece, c, sim)
        if att.tilt_margin < 1 or att.yaw_margin < 1:
            return False
        if c.worst_available_margin < Inputs.force_safety_factor:
            return False
        if prop.worst_acceleration_in_g < Inputs.min_maneuver_accel_g:
            return False
        if att.worst_tilt_margin < 1 or att.worst_yaw_margin < 1:
            return False
        psu = PowerSupply(coil, WireThermal(coil, c), TileControl(board, coil, Control(coil, c, sim)), c, driver, cooling)
        if psu.required_rating > psu.supply_rating:
            return False
        if psu.required_current > Fixed.max_bus_current:
            return False
        return True

    def __init__(self, board, coil, halbach, piece, sim):
        self.configurations = [
            CoilConfiguration(coil, halbach, piece, sim, conductor_thickness, bus_voltage, layers)
            for bus_voltage in Constants.standard_bus_voltages
            for conductor_thickness in Constants.winding_conductor_thicknesses
            for layers in range(1, floor(coil.outer_width / (conductor_thickness + 2 * Fixed.rectangular_wire_film)) + 1)
        ]
        self.feasible = [c for c in self.configurations if self.is_feasible(board, coil, halbach, piece, sim, c)]
        if not self.feasible:
            raise RuntimeError("NO FEASIBLE CONFIGURATION FOUND - no selected coil configuration")
        self.selected = min(self.feasible, key=self.copper_proxy)
        self.best_per_voltage = []
        for bus_voltage in Constants.standard_bus_voltages:
            candidates = [c for c in self.feasible if c.bus_voltage == bus_voltage]
            if candidates:
                self.best_per_voltage.append(min(candidates, key=self.copper_proxy))


class WireThermal:
    def __init__(self, coil, config):
        self.mass_per_winding = Constants.copper_density * config.length_per_winding * config.cross_section_area * 1000
        self.wire_length = config.length_per_winding * coil.windings
        self.copper_mass = self.mass_per_winding * coil.windings / 1000
        self.one_piece_power = config.piece_hover_power
        self.all_pieces_power = self.one_piece_power * Inputs.pieces_levitating_simultaneously
        self.psu_current = self.all_pieces_power / config.bus_voltage

    def cells(self):
        return [
            Cell("Copper mass per winding", self.mass_per_winding, "g"),
            Cell("Total wire length", self.wire_length, "m"),
            Cell("Total copper mass", self.copper_mass, "kg"),
            Cell("One-piece active coil power", self.one_piece_power, "W"),
            Cell("All board pieces simultaneous power", self.all_pieces_power, "W"),
            Cell("PSU current at bus", self.psu_current, "A"),
        ]


class DiscreteDriver:
    def __init__(self, board, coil, config):
        self.channels = coil.total_bodies
        self.half_bridges = self.channels * coil.half_bridges_per_coil
        self.active_channels = coil.active_windings
        self.current_feedback_channels = self.channels
        self.control_bits = self.channels * Fixed.current_command_bits_per_channel
        self.gate_drivers = ceil(self.half_bridges / Fixed.gate_driver_half_bridges)
        self.tile_count = ceil(board.motor_width / Fixed.control_tile_side) * ceil(board.motor_height / Fixed.control_tile_side)
        self.channels_per_tile = ceil(self.channels / self.tile_count)
        self.control_bits_per_tile = self.channels_per_tile * Fixed.current_command_bits_per_channel
        self.shift_registers_per_tile = ceil(self.control_bits_per_tile / Fixed.shift_register_outputs)
        self.shift_registers = self.shift_registers_per_tile * self.tile_count
        self.serial_data_rate = self.control_bits_per_tile * Fixed.current_command_rate
        self.serial_clock = min(Fixed.driver_serial_clock, Fixed.shift_register_clock_rating)
        self.serial_headroom = self.serial_clock / self.serial_data_rate
        self.current_squared_sum = config.piece_hover_power * Inputs.pieces_levitating_simultaneously / config.resistance
        self.current_sum_upper_bound = sqrt(self.active_channels * self.current_squared_sum)
        self.conduction_power = self.current_squared_sum * Fixed.driver_hot_resistance
        self.current_sense_power = self.current_squared_sum * Fixed.current_sense_resistance
        self.switching_power = (
            config.bus_voltage
            * self.current_sum_upper_bound
            * Fixed.driver_switching_time
            * Fixed.driver_pwm_frequency
        )
        self.gate_power = (
            self.active_channels
            * Fixed.driver_pwm_frequency
            * 2
            * Fixed.driver_mosfet_gate_charge
            * Fixed.gate_drive_voltage
        )
        self.control_power = (
            self.shift_registers
            * Fixed.shift_register_power_capacitance
            * Fixed.logic_gate_voltage ** 2
            * self.serial_data_rate
        )
        self.total_power = self.conduction_power + self.current_sense_power + self.switching_power + self.gate_power + self.control_power

    def cells(self):
        return [
            Cell("Driver implementation", "current-regulated 24V N-MOSFET full bridges"),
            Cell("Driver control implementation", "setpoint latch + resistor DAC + comparator current loop"),
            Cell("Dedicated driver channels", self.channels),
            Cell("Current feedback channels", self.current_feedback_channels),
            Cell("Discrete half-bridge legs", self.half_bridges),
            Cell("Gate-driver ICs", self.gate_drivers),
            Cell("Driver current limit", Fixed.driver_channel_current, "A"),
            Cell("MOSFET voltage rating", Fixed.mosfet_voltage_rating, "V"),
            Cell("Current-sense common-mode limit", Fixed.current_sense_common_mode_voltage, "V"),
            Cell("MOSFET gate drive", Fixed.gate_drive_voltage, "V"),
            Cell("PWM frequency", Fixed.driver_pwm_frequency / 1000, "kHz"),
            Cell("Current command bits/channel", Fixed.current_command_bits_per_channel),
            Cell("Current command refresh", Fixed.current_command_rate, "Hz"),
            Cell("Current loop bandwidth limit", Fixed.current_loop_bandwidth, "Hz"),
            Cell("Per-tile current-command stream", self.serial_data_rate / 1000000, "Mbit/s"),
            Cell("Control clock rating used", self.serial_clock / 1000000, "Mbit/s"),
            Cell("Control clock headroom", self.serial_headroom, "x"),
            Cell("Pessimistic hot path resistance", Fixed.driver_hot_resistance, "ohm"),
            Cell("Current shunt resistance", Fixed.current_sense_resistance, "ohm"),
            Cell("MOSFET conduction loss", self.conduction_power, "W"),
            Cell("Current-sense shunt loss", self.current_sense_power, "W"),
            Cell("MOSFET switching loss", self.switching_power, "W"),
            Cell("MOSFET gate-drive loss", self.gate_power, "W"),
            Cell("Shift-register dynamic loss", self.control_power, "W"),
            Cell("Total driver/control loss", self.total_power, "W"),
            Cell("Serialized control bits", self.control_bits),
            Cell("74HC595 shift registers", self.shift_registers),
        ]


class RadiatorCooling:
    def __init__(self, board, config, driver, fan_count):
        self.fan_count = fan_count
        self.mode = "fan-assisted" if fan_count else "passive"
        self.board_area = board.motor_area / 1000000
        self.source_area = min(self.board_area, pi / 4 * (board.base_diameter / 1000) ** 2 * Inputs.pieces_levitating_simultaneously)
        self.source_to_baseplate_resistance = (
            Fixed.potting_thickness / 1000 / Fixed.potting_thermal_conductivity
            + Fixed.pcb_thickness / 1000 / Fixed.pcb_via_effective_thermal_conductivity
            + Fixed.thermal_pad_thickness / 1000 / Fixed.thermal_pad_conductivity
            + Fixed.baseplate_thickness / 1000 / Fixed.aluminium_thermal_conductivity
        ) / self.source_area
        self.fin_pitch = (Fixed.fin_thickness + Fixed.fin_channel_width) / 1000
        self.fin_count = max(1, floor((board.motor_width / 1000 + Fixed.fin_channel_width / 1000) / self.fin_pitch))
        self.fin_length = board.motor_height / 1000
        self.fin_height = Fixed.fin_height / 1000
        self.fin_thickness = Fixed.fin_thickness / 1000
        self.fin_footprint_area = self.fin_count * self.fin_thickness * self.fin_length
        self.convection_area = self.board_area + 2 * self.fin_count * self.fin_height * self.fin_length
        self.convection_coefficient = Fixed.forced_convection_coefficient if fan_count else Fixed.natural_convection_coefficient
        self.thermal_conductance = self.convection_coefficient * self.convection_area
        self.baseplate_volume = self.board_area * Fixed.baseplate_thickness / 1000
        self.fin_volume = self.fin_footprint_area * self.fin_height
        self.aluminium_mass = (self.baseplate_volume + self.fin_volume) * Fixed.aluminium_density
        self.thermal_capacitance = self.aluminium_mass * Fixed.aluminium_heat_capacity
        self.thermal_time_constant = self.thermal_capacitance / self.thermal_conductance
        self.coil_power = config.piece_hover_power * Inputs.pieces_levitating_simultaneously
        self.driver_power = driver.total_power
        self.pulse_power = self.coil_power + self.driver_power
        self.period = Inputs.max_hover_duration + Inputs.spot_cooldown_duration
        self.duty_cycle = Inputs.max_hover_duration / self.period
        self.baseplate_rise_per_watt = (
            1 / self.thermal_conductance
            * (1 - exp(-Inputs.max_hover_duration / self.thermal_time_constant))
            / (1 - exp(-self.period / self.thermal_time_constant))
        )
        self.cyclic_peak_baseplate_rise = self.pulse_power * self.baseplate_rise_per_watt
        self.cyclic_peak_baseplate_temp = Inputs.ambient_temperature + self.cyclic_peak_baseplate_rise
        self.cyclic_peak_source_temp = self.cyclic_peak_baseplate_temp + self.pulse_power * self.source_to_baseplate_resistance
        self.source_rise_per_watt = self.baseplate_rise_per_watt + self.source_to_baseplate_resistance
        self.baseplate_power_capacity = (Inputs.max_surface_temperature - Inputs.ambient_temperature) / self.baseplate_rise_per_watt
        self.source_power_capacity = Inputs.allowed_wire_temp_rise / self.source_rise_per_watt
        self.thermal_power_capacity = min(self.baseplate_power_capacity, self.source_power_capacity)
        self.driver_heat_capacity = max(0, self.thermal_power_capacity - self.coil_power)
        self.thermal_power_margin = self.thermal_power_capacity / self.pulse_power
        self.fan_bank_width = fan_count * Fixed.cooling_fan_size
        self.fan_bank_fits = self.fan_bank_width <= board.motor_width and Fixed.cooling_fan_size <= board.motor_height
        self.rated_airflow = fan_count * Fixed.cooling_fan_airflow
        self.effective_airflow = self.rated_airflow * Fixed.cooling_fan_airflow_fraction
        self.fan_power = fan_count * Fixed.cooling_fan_power
        self.fan_mass = fan_count * Fixed.cooling_fan_mass
        self.fan_noise = 0 if not fan_count else Fixed.cooling_fan_noise + 10 * log10(fan_count) + Fixed.cooling_fan_installation_noise

    def cells(self):
        cells = [
            Cell("Cooling mode", self.mode),
            Cell("Coil heat (all pieces)", self.coil_power, "W"),
            Cell("Driver/control heat", self.driver_power, "W"),
            Cell("Total pulse heat", self.pulse_power, "W"),
            Cell("Source area (32 piece footprints)", self.source_area * 10000, "cm2"),
            Cell("Source-to-baseplate resistance", self.source_to_baseplate_resistance, "K/W"),
            Cell("Bottom fins", self.fin_count),
            Cell("Fin height", self.fin_height * 1000, "mm"),
            Cell("Fin channel width", Fixed.fin_channel_width, "mm"),
            Cell("Effective convection area", self.convection_area, "m2"),
            Cell("Convection coefficient", self.convection_coefficient, "W/(m2.K)"),
            Cell("Radiator aluminium mass", self.aluminium_mass, "kg"),
            Cell("Thermal time constant", self.thermal_time_constant, "s"),
            Cell("All-piece duty cycle (2s/60s)", self.duty_cycle),
            Cell("Cyclic peak baseplate temp", self.cyclic_peak_baseplate_temp, "C"),
            Cell("Cyclic peak coil/MOSFET temp", self.cyclic_peak_source_temp, "C"),
            Cell("Maximum total pulse heat", self.thermal_power_capacity, "W"),
            Cell("Maximum driver/control heat after coils", self.driver_heat_capacity, "W"),
            Cell("Thermal power margin", self.thermal_power_margin, "x"),
        ]
        if self.fan_count:
            cells.extend([
                Cell("Cooling fans", self.fan_count),
                Cell("Fan size", Fixed.cooling_fan_size, "mm"),
                Cell("Fan speed", Fixed.cooling_fan_speed, "rpm"),
                Cell("Fan row width", self.fan_bank_width, "mm"),
                Cell("Fan row fits radiator", "yes" if self.fan_bank_fits else "no"),
                Cell("Rated fan airflow", self.rated_airflow, "m3/h"),
                Cell("Assumed effective airflow", self.effective_airflow, "m3/h"),
                Cell("Low-noise static pressure", Fixed.cooling_fan_static_pressure, "mm H2O"),
                Cell("Fan electrical power", self.fan_power, "W"),
                Cell("Installed fan-noise estimate", self.fan_noise, "dB(A)"),
            ])
        return cells


class Propulsion:
    def __init__(self, board, coil, piece, config, halbach, sim):
        self.mass = piece.mass / 1000
        self.weight = piece.weight
        self.available_force = config.available_force
        self.lateral_force_per_amp = config.force_per_amp * sim["lateral_to_lift_ratio"]
        self.max_thrust = config.current_limit * self.lateral_force_per_amp
        self.max_acceleration = self.max_thrust / self.mass
        self.acceleration_in_g = self.max_acceleration / Constants.gravity
        self.worst_lateral_force_per_amp = config.turns * sim["worst_lateral_force_per_ampere_turn"] * config.height_coupling
        self.worst_max_thrust = config.current_limit * self.worst_lateral_force_per_amp
        self.worst_acceleration_in_g = self.worst_max_thrust / self.mass / Constants.gravity
        self.thrust_current = self.max_thrust / self.lateral_force_per_amp
        self.flight_power = config.current_limit ** 2 * config.resistance * coil.control_bed_bodies
        self.square_pitch = board.square_size / 1000
        self.hop_time = 2 * sqrt(self.square_pitch / self.max_acceleration)
        self.hop_peak_speed = sqrt(self.max_acceleration * self.square_pitch)
        self.traverse_distance = (Constants.board_squares_per_side - 1) * self.square_pitch
        self.traverse_time = 2 * sqrt(self.traverse_distance / self.max_acceleration)
        self.magnet_span = board.platform_side / 1000
        self.yaw_inertia = halbach.magnet_mass / 1000 * self.magnet_span ** 2 / 6
        self.yaw_torque = self.max_thrust * self.magnet_span / 4
        self.yaw_acceleration = self.yaw_torque / self.yaw_inertia

    def cells(self):
        return [
            Cell("Max lateral thrust", self.max_thrust, "N"),
            Cell("Max lateral acceleration", self.max_acceleration, "m/s2"),
            Cell("Max lateral acceleration", self.acceleration_in_g, "g"),
            Cell("Worst-case max lateral acceleration", self.worst_acceleration_in_g, "g"),
            Cell("Thrust current per winding", self.thrust_current, "A"),
            Cell("In-flight coil power (one piece)", self.flight_power, "W"),
            Cell("One-square hop time (bang-bang)", self.hop_time, "s"),
            Cell("One-square peak speed", self.hop_peak_speed, "m/s"),
            Cell("Full-rank traverse time", self.traverse_time, "s"),
            Cell("Yaw moment of inertia", self.yaw_inertia, "kg.m2"),
            Cell("Max yaw torque", self.yaw_torque, "N.m"),
            Cell("Max yaw angular acceleration", self.yaw_acceleration, "rad/s2"),
        ]


class AttitudeAuthority:
    def __init__(self, board, piece, config, sim):
        self.mass = piece.mass / 1000
        self.radius = piece.diameter / 2 / 1000
        self.height = piece.box_height / 1000
        self.com_height = Fixed.com_height_fraction * self.height
        self.lever_arm = board.platform_side / 4 / 1000
        self.tilt_torque_max = sim["tilt_torque_per_ampere_turn"] * config.turns * config.current_limit * config.height_coupling
        self.tilt_angle = radians(Inputs.target_tilt_angle_deg)
        self.tilt_inertia = self.mass * (3 * self.radius ** 2 + self.height ** 2) / 12
        self.tilt_accel = 4 * self.tilt_angle / Inputs.target_tilt_time ** 2
        self.tilt_static_torque = self.mass * Constants.gravity * self.com_height * sin(self.tilt_angle)
        self.tilt_dynamic_torque = self.tilt_inertia * self.tilt_accel
        self.tilt_required_torque = self.tilt_static_torque + self.tilt_dynamic_torque
        self.tilt_margin = self.tilt_torque_max / self.tilt_required_torque
        self.worst_tilt_torque_max = sim["worst_tilt_torque_per_ampere_turn"] * config.turns * config.current_limit * config.height_coupling
        self.worst_tilt_margin = self.worst_tilt_torque_max / self.tilt_required_torque
        self.yaw_torque_max = sim["yaw_torque_per_ampere_turn"] * config.turns * config.current_limit * config.height_coupling
        self.yaw_angle = radians(Inputs.target_yaw_angle_deg)
        self.yaw_inertia = self.mass * self.radius ** 2 / 2
        self.yaw_accel = 4 * self.yaw_angle / Inputs.target_yaw_time ** 2
        self.yaw_required_torque = self.yaw_inertia * self.yaw_accel
        self.yaw_margin = self.yaw_torque_max / self.yaw_required_torque
        self.worst_yaw_torque_max = sim["worst_yaw_torque_per_ampere_turn"] * config.turns * config.current_limit * config.height_coupling
        self.worst_yaw_margin = self.worst_yaw_torque_max / self.yaw_required_torque

    def cells(self):
        return [
            Cell("Tilt lever arm (footprint)", self.lever_arm * 1000, "mm"),
            Cell("Max tilt torque available (sim)", self.tilt_torque_max, "N.m"),
            Cell("Tilt inertia about diameter", self.tilt_inertia, "kg.m2"),
            Cell("Static torque to hold target tilt", self.tilt_static_torque, "N.m"),
            Cell("Dynamic torque to reach tilt in time", self.tilt_dynamic_torque, "N.m"),
            Cell("Tilt torque required (hold+slew)", self.tilt_required_torque, "N.m"),
            Cell("Tilt authority margin", self.tilt_margin, "x"),
            Cell("Worst-case tilt authority margin", self.worst_tilt_margin, "x"),
            Cell("Max yaw torque available (sim)", self.yaw_torque_max, "N.m"),
            Cell("Yaw inertia about vertical", self.yaw_inertia, "kg.m2"),
            Cell("Yaw torque required for 90deg slew", self.yaw_required_torque, "N.m"),
            Cell("Yaw authority margin", self.yaw_margin, "x"),
            Cell("Worst-case yaw authority margin", self.worst_yaw_margin, "x"),
        ]


class Control:
    def coil_inductance(self, turns, footprint_area, height):
        return Constants.vacuum_permeability * turns ** 2 * (footprint_area / 1000000) / (height / 1000) * 1000

    def __init__(self, coil, config, sim):
        self.inductance = self.coil_inductance(config.turns, coil.footprint_area, config.coil_height)
        self.time_constant = (self.inductance / 1000) / config.resistance * 1000
        self.electrical_bandwidth = 1 / (2 * pi * (self.time_constant / 1000))
        self.actuator_bandwidth = min(self.electrical_bandwidth, Fixed.current_loop_bandwidth)
        self.slew_time = (self.inductance / 1000) * config.operating_current / config.usable_drive_voltage * 1000
        self.instability_time = sim["instability_growth_time"] * 1000
        self.required_bandwidth = Inputs.control_loop_bandwidth_margin / (2 * pi * (self.instability_time / 1000))
        self.pose_update_rate = Inputs.control_loop_bandwidth_margin / (self.instability_time / 1000)

    def cells(self):
        return [
            Cell("Coil inductance (estimate)", self.inductance, "mH"),
            Cell("Electrical time constant L/R", self.time_constant, "ms"),
            Cell("Electrical L/R bandwidth", self.electrical_bandwidth, "Hz"),
            Cell("Closed-loop current bandwidth", self.actuator_bandwidth, "Hz"),
            Cell("Current slew time to Imax", self.slew_time, "ms"),
            Cell("Open-loop instability growth time", self.instability_time, "ms"),
            Cell("Required control loop bandwidth", self.required_bandwidth, "Hz"),
            Cell("Required pose update rate", self.pose_update_rate, "Hz"),
        ]


class DriveMatrix:
    def __init__(self, coil, control, driver, tiles):
        self.scheme = "dedicated full H-bridge per coil (bipolar, full Vbus swing)"
        self.half_bridges_per_coil = coil.half_bridges_per_coil
        self.driver_half_bridges = driver.half_bridges
        self.control_bits = driver.control_bits
        self.shift_registers = driver.shift_registers
        self.current_feedback_channels = driver.current_feedback_channels
        self.control_bits_per_tile = driver.control_bits_per_tile
        self.serial_data_rate = driver.serial_data_rate
        self.serial_clock = driver.serial_clock
        self.serial_headroom = driver.serial_headroom
        self.current_command_rate_headroom = Fixed.current_command_rate / control.pose_update_rate
        self.current_loop_headroom = control.actuator_bandwidth / control.required_bandwidth
        self.current_pwm_loop_cycles = Fixed.driver_pwm_frequency / Fixed.current_loop_bandwidth
        self.current_command_resolution = Fixed.driver_channel_current / (2 ** Fixed.current_pwm_resolution_bits)
        self.current_monitor_reads_per_tile = driver.channels_per_tile * control.pose_update_rate
        self.current_monitor_headroom = Fixed.current_monitor_adc_sample_rate / self.current_monitor_reads_per_tile
        self.coils_energized = coil.peak_driven_windings
        self.slew_time = control.slew_time
        self.update_period = 1000 / control.pose_update_rate
        self.slew_over_update = self.slew_time / self.update_period

    def cells(self):
        return [
            Cell("Drive scheme", self.scheme),
            Cell("Half-bridges per coil", self.half_bridges_per_coil),
            Cell("Driver half-bridges provided", self.driver_half_bridges),
            Cell("Serialized control bits", self.control_bits),
            Cell("Current feedback channels", self.current_feedback_channels),
            Cell("74HC595 shift registers", self.shift_registers),
            Cell("Per-tile current-command data", self.serial_data_rate / 1000000, "Mbit/s"),
            Cell("Per-tile SPI clock", self.serial_clock / 1000000, "Mbit/s"),
            Cell("Current-command serial headroom", self.serial_headroom, "x"),
            Cell("Current command-rate headroom", self.current_command_rate_headroom, "x"),
            Cell("Current-loop bandwidth headroom", self.current_loop_headroom, "x"),
            Cell("PWM cycles per current-loop bandwidth", self.current_pwm_loop_cycles),
            Cell("Current command resolution", self.current_command_resolution * 1000, "mA"),
            Cell("Current-monitor reads per tile", self.current_monitor_reads_per_tile, "samples/s"),
            Cell("Current-monitor ADC headroom", self.current_monitor_headroom, "x"),
            Cell("Coils energized at once", self.coils_energized),
            Cell("Current slew time to Imax", self.slew_time, "ms"),
            Cell("Control update period", self.update_period, "ms"),
            Cell("Slew / update-period ratio", self.slew_over_update, "x"),
        ]


class HallSensing:
    def __init__(self, board, control, tiles, sim):
        self.update_rate = control.pose_update_rate
        self.sensor_pitch = Fixed.hall_sensor_pitch
        self.sensors_per_piece = Fixed.hall_observation_window_side ** 2
        self.sensors_per_tile_side = ceil(Fixed.control_tile_side / self.sensor_pitch)
        self.sensors_per_tile = self.sensors_per_tile_side ** 2
        self.total_sensors = self.sensors_per_tile * tiles.tile_count
        self.muxes_per_tile = ceil(self.sensors_per_tile / Fixed.hall_sensor_mux_channels)
        self.reads_per_tile = self.sensors_per_tile * self.update_rate
        self.tile_capacity = Fixed.hall_adc_sample_rate
        self.headroom = self.tile_capacity / self.reads_per_tile
        self.position_resolution_um = self.sensor_pitch * 1000 / (2 ** Fixed.hall_interpolation_bits)
        self.nominal_rank = sim["hall_rank6"]
        self.nominal_condition = sim["hall_condition6"]
        self.worst_rank = sim["hall_worst_rank6"]
        self.worst_condition = sim["hall_worst_condition6"]
        self.worst_poses = sim["hall_worst_case_poses"]
        self.observation_window_side = sim["hall_observation_window_side"]

    def cells(self):
        return [
            Cell("Required update rate", self.update_rate, "Hz"),
            Cell("Hall sensor pitch", self.sensor_pitch, "mm"),
            Cell("Estimator observation window", f"{self.observation_window_side}x{self.observation_window_side}"),
            Cell("Sensors used per piece estimate", self.sensors_per_piece),
            Cell("Fixed-grid worst-case poses", self.worst_poses),
            Cell("Nominal Hall observability rank", self.nominal_rank),
            Cell("Nominal Hall condition", self.nominal_condition, "x"),
            Cell("Worst fixed-grid Hall rank", self.worst_rank),
            Cell("Worst fixed-grid Hall condition", self.worst_condition, "x"),
            Cell("Sensors per tile", self.sensors_per_tile),
            Cell("Total Hall sensors (board)", self.total_sensors),
            Cell("Readout muxes per tile", self.muxes_per_tile),
            Cell("Reads needed per tile", self.reads_per_tile, "reads/s"),
            Cell("Per-tile ADC capacity", self.tile_capacity, "samples/s"),
            Cell("Sensing headroom", self.headroom, "x"),
            Cell("Interpolated position resolution", self.position_resolution_um, "um"),
        ]


class TileControl:
    def __init__(self, board, coil, control):
        self.tile_side = Fixed.control_tile_side
        self.tiles_per_width = ceil(board.motor_width / self.tile_side)
        self.tiles_per_height = ceil(board.motor_height / self.tile_side)
        self.tile_count = self.tiles_per_width * self.tiles_per_height
        self.coils_per_tile = ceil(coil.total_bodies / self.tile_count)
        self.square_area = board.square_size ** 2
        self.max_pieces_per_tile = max(1, ceil(self.tile_side ** 2 / self.square_area))
        self.pose_rate = control.pose_update_rate
        self.node_capacity = Fixed.node_mcu_throughput_mflops * 1e6
        self.tile_compute = self.max_pieces_per_tile * Fixed.piece_control_flops * self.pose_rate
        self.central_compute = Inputs.pieces_levitating_simultaneously * Fixed.piece_control_flops * self.pose_rate
        self.tile_headroom = self.node_capacity / self.tile_compute
        self.central_headroom = self.node_capacity / self.central_compute

    def cells(self):
        return [
            Cell("Control tile side (square)", self.tile_side, "mm"),
            Cell("Control tiles (count)", self.tile_count),
            Cell("Coils per tile", self.coils_per_tile),
            Cell("Max pieces over one tile", self.max_pieces_per_tile),
            Cell("Per-tile compute load", self.tile_compute / 1e6, "Mflop/s"),
            Cell("Per-tile MCU capacity", self.node_capacity / 1e6, "Mflop/s"),
            Cell("Per-tile compute headroom", self.tile_headroom, "x"),
            Cell("Single-MCU compute headroom (rejected)", self.central_headroom, "x"),
        ]


class PowerSupply:
    def __init__(self, coil, wire, tiles, config, driver, thermal):
        self.bus_voltage = config.bus_voltage
        self.coil_lift_power = wire.all_pieces_power
        self.thrust_factor = coil.peak_driven_windings / coil.active_windings
        self.coil_peak_power = self.coil_lift_power * self.thrust_factor
        self.driver_peak_power = driver.total_power * self.thrust_factor
        self.electronics_power = (tiles.tile_count * Fixed.tile_mcu_power
                                  + Fixed.host_power
                                  + thermal.fan_power)
        self.total_load = self.coil_peak_power + self.driver_peak_power + self.electronics_power
        self.required_rating = self.total_load * Fixed.psu_sizing_margin
        self.psu_part, self.supply_rating, self.psu_price, self.psu_url = Fixed.psu_options[config.bus_voltage]
        self.peak_current = self.total_load / self.bus_voltage
        self.required_current = self.required_rating / self.bus_voltage
        self.rated_current = self.supply_rating / self.bus_voltage
        self.load_fraction = self.required_rating / self.supply_rating

    def cells(self):
        return [
            Cell("Coil lift power (32 pieces)", self.coil_lift_power, "W"),
            Cell("Coil peak power (+thrust)", self.coil_peak_power, "W"),
            Cell("Discrete driver peak loss", self.driver_peak_power, "W"),
            Cell("Electronics overhead", self.electronics_power, "W"),
            Cell("Total peak load", self.total_load, "W"),
            Cell("Required PSU rating (+margin)", self.required_rating, "W"),
            Cell(f"Selected PSU ({self.psu_part})", self.supply_rating, "W"),
            Cell("Peak bus current", self.peak_current, "A"),
            Cell("Required bus current (+margin)", self.required_current, "A"),
            Cell("PSU output current", self.rated_current, "A"),
            Cell("PSU load fraction", self.load_fraction, "x"),
        ]


class Stability:
    def __init__(self, board, piece, control, sim):
        self.mass = piece.mass / 1000
        self.height = piece.box_height / 1000
        self.half_width = board.platform_side / 2 / 1000
        self.vertical_stiffness = sim["vertical_stiffness"]
        self.bounce_frequency = sqrt(self.vertical_stiffness / self.mass) / (2 * pi)
        self.tilt_stiffness = self.vertical_stiffness * self.half_width ** 2
        self.tilt_inertia = self.mass * self.height ** 2 / 3
        self.rock_frequency = sqrt(self.tilt_stiffness / self.tilt_inertia) / (2 * pi)
        self.control_margin_over_rock = control.required_bandwidth / self.rock_frequency
        self.sense_resolution = Inputs.position_sense_resolution_um / 1e6
        self.sense_baseline = board.platform_side / 1000
        self.tilt_sense_resolution = self.sense_resolution / self.sense_baseline
        self.tip_sense_resolution = self.tilt_sense_resolution * self.height

    def cells(self):
        return [
            Cell("Vertical magnetic stiffness", self.vertical_stiffness, "N/m"),
            Cell("Vertical bounce frequency", self.bounce_frequency, "Hz"),
            Cell("Tilt (rock) stiffness", self.tilt_stiffness, "N.m/rad"),
            Cell("Tip moment of inertia", self.tilt_inertia, "kg.m2"),
            Cell("King rocking frequency", self.rock_frequency, "Hz"),
            Cell("Control bandwidth over rock mode", self.control_margin_over_rock, "x"),
            Cell("Position sense resolution", Inputs.position_sense_resolution_um, "um"),
            Cell("Tilt sense baseline", self.sense_baseline * 1000, "mm"),
            Cell("Tilt sense resolution", self.tilt_sense_resolution * 1000, "mrad"),
            Cell("Tip position sense resolution", self.tip_sense_resolution * 1e6, "um"),
        ]


class StatusChecks:
    def passes(self, condition, ok_text, fail_text):
        return ok_text if condition else fail_text

    def __init__(self, board, coil, halbach, piece, snap, config, control, sensing, thermal, propulsion, attitude, stability, drive, tiles, psu, sim):
        self.force = self.passes(config.available_margin >= 1, "OK", "not enough force")
        self.safety = self.passes(config.available_margin >= Inputs.force_safety_factor, "OK", "below safety margin")
        self.voltage = self.passes(config.voltage_per_winding <= config.usable_drive_voltage, "OK", "voltage too high")
        self.baseplate_thermal = self.passes(thermal.cyclic_peak_baseplate_temp <= Inputs.max_surface_temperature, "OK", "baseplate too hot")
        self.source_thermal = self.passes(thermal.cyclic_peak_source_temp <= Inputs.ambient_temperature + Inputs.allowed_wire_temp_rise, "OK", "coil/MOSFET source too hot")
        self.maneuvering = self.passes(propulsion.acceleration_in_g >= Inputs.min_maneuver_accel_g, "OK", "lateral thrust too weak")
        self.tilt_authority = self.passes(attitude.tilt_margin >= 1, "OK", "not enough tilt torque")
        self.yaw_authority = self.passes(attitude.yaw_margin >= 1, "OK", "not enough yaw torque")
        self.worst_force = self.passes(config.worst_available_margin >= Inputs.force_safety_factor, "OK", "worst-case pose lift below safety margin")
        self.worst_maneuvering = self.passes(propulsion.worst_acceleration_in_g >= Inputs.min_maneuver_accel_g, "OK", "worst-case lateral thrust too weak")
        self.worst_tilt_authority = self.passes(attitude.worst_tilt_margin >= 1, "OK", "worst-case tilt torque too weak")
        self.worst_yaw_authority = self.passes(attitude.worst_yaw_margin >= 1, "OK", "worst-case yaw torque too weak")
        self.rock_controllable = self.passes(stability.control_margin_over_rock >= Inputs.control_loop_bandwidth_margin, "OK", "rock mode too fast for loop")
        self.tilt_observable = self.passes(stability.tip_sense_resolution <= 0.001, "OK", "tilt sensing too coarse")
        self.driver_voltage = self.passes(config.bus_voltage <= Fixed.mosfet_voltage_rating, "OK", "bus exceeds MOSFET voltage rating")
        self.current_sense_voltage = self.passes(config.bus_voltage <= Fixed.current_sense_common_mode_voltage, "OK", "bus exceeds current-sense common-mode rating")
        self.driver_current = self.passes(config.worst_required_current <= Fixed.driver_channel_current, "OK", "required coil current exceeds channel rating")
        self.actuator_rank = self.passes(sim["actuator_rank6"] >= 6, "OK", "actuator matrix not full 6-DOF rank")
        self.hall_observable = self.passes(sim["hall_worst_rank6"] >= 6, "OK", "Hall array cannot observe all 6 DOF across fixed-grid phases")
        self.shell_validity = self.passes((piece.diameter - 2 * Inputs.plastic_wall_thickness) > 0, "OK", "wall too thick")
        self.coil_height = self.passes(config.coil_height <= coil.outer_width, "OK", "coil too tall")
        self.coil_window = self.passes(config.inner_window_width > 0 and config.inner_window_length > 0, "OK", "winding walls overlap, no coil opening")
        self.platform_size = self.passes(20 <= board.platform_side <= 50, "OK", "platform out of range")
        self.magnet_fits_base = self.passes(board.platform_side <= board.base_diameter, "OK", "magnet array wider than base")
        self.neighbour_snap = self.passes(snap.snap_to_weight <= snap.holding_friction, "OK", "resting pieces magnetically snap")
        self.control_bandwidth = self.passes(control.actuator_bandwidth >= control.required_bandwidth, "OK", "actuator bandwidth too low")
        self.current_slew = self.passes(control.slew_time <= control.instability_time / Inputs.control_loop_bandwidth_margin, "OK", "current cannot react in time")
        self.hall_throughput = self.passes(sensing.headroom >= 1, "OK", "tile ADC too slow to scan Hall grid")
        self.hall_resolution = self.passes(sensing.position_resolution_um <= Inputs.position_sense_resolution_um, "OK", "Hall grid too coarse for position resolution")
        self.driver_coverage = self.passes(drive.driver_half_bridges >= coil.total_bodies * coil.half_bridges_per_coil, "OK", "not enough driver half-bridges per coil")
        self.current_feedback = self.passes(drive.current_feedback_channels >= coil.total_bodies, "OK", "not every coil has current feedback")
        self.current_command_rate = self.passes(drive.current_command_rate_headroom >= 1, "OK", "current command update too slow")
        self.current_loop = self.passes(drive.current_loop_headroom >= 1, "OK", "current loop too slow")
        self.current_pwm_loop = self.passes(drive.current_pwm_loop_cycles >= Fixed.min_current_loop_pwm_cycles, "OK", "PWM frequency too low for current loop")
        self.current_resolution = self.passes(drive.current_command_resolution <= Fixed.max_current_command_error, "OK", "current command resolution too coarse")
        self.current_monitor = self.passes(drive.current_monitor_headroom >= 1, "OK", "current monitor ADC too slow")
        self.driver_serial = self.passes(drive.serial_headroom >= 1, "OK", "current-command stream exceeds SPI clock")
        self.drive_slew = self.passes(drive.slew_over_update <= 1, "OK", "current too slow for update period")
        self.tile_compute = self.passes(tiles.tile_headroom >= 1, "OK", "tile MCU overloaded")
        self.cooling_fans_fit = self.passes(thermal.fan_bank_fits, "OK", "cooling fan row does not fit radiator")
        self.psu_adequate = self.passes(psu.required_rating <= psu.supply_rating, "OK", "PSU undersized")
        self.bus_current = self.passes(psu.required_current <= Fixed.max_bus_current, "OK", "bus current too high")

    def cells(self):
        return [
            Cell("Force check", self.force),
            Cell("Safety-margin check", self.safety),
            Cell("Voltage check", self.voltage),
            Cell("Baseplate thermal check", self.baseplate_thermal),
            Cell("Coil/MOSFET thermal check", self.source_thermal),
            Cell("Maneuvering check", self.maneuvering),
            Cell("Tilt-authority check", self.tilt_authority),
            Cell("Yaw-authority check", self.yaw_authority),
            Cell("Worst-case-pose lift check", self.worst_force),
            Cell("Worst-case maneuvering check", self.worst_maneuvering),
            Cell("Worst-case tilt-authority check", self.worst_tilt_authority),
            Cell("Worst-case yaw-authority check", self.worst_yaw_authority),
            Cell("Rock-mode controllable check", self.rock_controllable),
            Cell("Tilt-observable check", self.tilt_observable),
            Cell("Driver-bus-voltage check", self.driver_voltage),
            Cell("Current-sense-voltage check", self.current_sense_voltage),
            Cell("Driver-channel-current check", self.driver_current),
            Cell("Actuator 6-DOF rank check", self.actuator_rank),
            Cell("Hall 6-DOF observability check", self.hall_observable),
            Cell("Shell-validity check", self.shell_validity),
            Cell("Coil-height buildable check", self.coil_height),
            Cell("Coil-window non-degenerate check", self.coil_window),
            Cell("Platform-size check", self.platform_size),
            Cell("Magnet-array-fits-base check", self.magnet_fits_base),
            Cell("Neighbour-snap check", self.neighbour_snap),
            Cell("Control-bandwidth check", self.control_bandwidth),
            Cell("Current-slew check", self.current_slew),
            Cell("Hall-throughput check", self.hall_throughput),
            Cell("Hall-grid-resolution check", self.hall_resolution),
            Cell("Driver-coverage check", self.driver_coverage),
            Cell("Current-feedback-coverage check", self.current_feedback),
            Cell("Current-command-rate check", self.current_command_rate),
            Cell("Current-loop-bandwidth check", self.current_loop),
            Cell("Current-loop-PWM check", self.current_pwm_loop),
            Cell("Current-command-resolution check", self.current_resolution),
            Cell("Current-monitor-throughput check", self.current_monitor),
            Cell("Driver-serial-throughput check", self.driver_serial),
            Cell("Drive-slew check", self.drive_slew),
            Cell("Per-tile-compute check", self.tile_compute),
            Cell("Cooling-fan-fit check", self.cooling_fans_fit),
            Cell("PSU-adequate check", self.psu_adequate),
            Cell("Bus-current check", self.bus_current),
        ]


class BomItem:
    def __init__(self, scope, category, spec, qty_per_unit, unit_cost, link=""):
        self.scope = scope
        self.category = category
        self.spec = spec
        self.qty_per_unit = qty_per_unit
        self.unit_cost = unit_cost
        self.link = link
        self.line_cost = qty_per_unit * unit_cost


class MassBudget:
    def __init__(self, board, wire, piece, thermal):
        self.board_copper_mass = wire.copper_mass
        self.board_pcb_mass = board.motor_area * Fixed.pcb_thickness * Constants.fr4_density / 1000
        self.radiator_mass = thermal.aluminium_mass
        self.cooling_fan_mass = thermal.fan_mass
        self.board_added_mass = Fixed.psu_mass_kg + Fixed.frame_enclosure_mass_kg + Fixed.board_electronics_mass_kg
        self.board_total_mass = self.board_copper_mass + self.board_pcb_mass + self.radiator_mass + self.cooling_fan_mass + self.board_added_mass
        self.piece_mass = piece.mass / 1000
        self.pieces_total = Fixed.captured_pieces_total
        self.all_pieces_mass = self.piece_mass * self.pieces_total
        self.set_total_mass = self.board_total_mass + self.all_pieces_mass

    def cells(self):
        return [
            Cell("Board copper (coils)", self.board_copper_mass, "kg"),
            Cell("Board PCB (FR4)", self.board_pcb_mass, "kg"),
            Cell("Aluminium baseplate + fins", self.radiator_mass, "kg"),
            Cell("Cooling fans", self.cooling_fan_mass, "kg"),
            Cell("PSU + frame + electronics (est.)", self.board_added_mass, "kg"),
            Cell("Board total (est.)", self.board_total_mass, "kg"),
            Cell("Mass per piece", self.piece_mass * 1000, "g"),
            Cell("Pieces total", self.pieces_total),
            Cell("All pieces mass", self.all_pieces_mass, "kg"),
            Cell("WHOLE SET mass (est.)", self.set_total_mass, "kg"),
        ]


class BillOfMaterials:
    def __init__(self, board, coil, halbach, wire, config, tiles, sensing, thermal):
        coils_per_tile = ceil(coil.total_bodies / tiles.tile_count)
        half_bridges_per_tile = coils_per_tile * coil.half_bridges_per_coil
        tile_power_mosfets = half_bridges_per_tile * 2
        tile_gate_drivers = ceil(half_bridges_per_tile / Fixed.gate_driver_half_bridges)
        tile_shift_registers = ceil(coils_per_tile * Fixed.current_command_bits_per_channel / Fixed.shift_register_outputs)
        tile_current_setpoint_passives = coils_per_tile * Fixed.current_setpoint_dac_passives_per_channel
        tile_current_shunts = coils_per_tile
        tile_current_sense_amps = coils_per_tile
        tile_current_comparators = ceil(coils_per_tile / Fixed.current_comparator_channels_per_ic)
        tile_current_frontend_passives = coils_per_tile * Fixed.current_frontend_passives_per_channel
        tile_driver_passives = half_bridges_per_tile * 2
        tile_driver_decoupling = tile_shift_registers + tile_gate_drivers + tile_current_sense_amps + tile_current_comparators
        tile_driver_solder_joints = (
            tile_power_mosfets * 3
            + tile_gate_drivers * 20
            + tile_shift_registers * 16
            + tile_current_setpoint_passives * 2
            + tile_current_shunts * 2
            + tile_current_sense_amps * 6
            + tile_current_comparators * 8
            + tile_current_frontend_passives * 2
            + tile_driver_passives * 2
            + tile_driver_decoupling * 2
        )
        tile_hall_sensors = sensing.sensors_per_tile
        tile_hall_muxes = sensing.muxes_per_tile
        tile_wire_kg = wire.copper_mass / tiles.tile_count
        tile_pcb_area_cm2 = (tiles.tile_side ** 2) / 100

        self.tile_count = tiles.tile_count
        self.piece_count = Inputs.pieces_levitating_simultaneously
        self.coils_per_tile = coils_per_tile

        self.tile_items = [
            BomItem("tile", "Driver power MOSFET", "TECH PUBLIC 20N06 60V N-MOSFET (LCSC C5350878)", tile_power_mosfets, Fixed.power_mosfet_price, "https://www.lcsc.com/product-detail/mosfets_tech-public-20n06_C5350878.html"),
            BomItem("tile", "Driver gate driver", "EG Micro EG2134 3 half-bridge MOSFET driver (LCSC C480661)", tile_gate_drivers, Fixed.gate_driver_price, "https://www.lcsc.com/product-detail/C480661.html"),
            BomItem("tile", "Current setpoint latch", "Gcore GR74HC595 8-bit shift register (LCSC C18164493)", tile_shift_registers, Fixed.shift_register_price, "https://www.lcsc.com/product-detail/C18164493.html"),
            BomItem("tile", "Current setpoint DAC", "12-bit resistor DAC reference network", tile_current_setpoint_passives, Fixed.current_frontend_passive_price),
            BomItem("tile", "Current shunt", "20mohm 2512 current sense resistor (LCSC C2985717)", tile_current_shunts, Fixed.current_shunt_price, "https://www.lcsc.com/product-detail/C2985717.html"),
            BomItem("tile", "Current sense amp", "TI INA181A2IDBVR bidirectional current-sense amp (LCSC C2058784)", tile_current_sense_amps, Fixed.current_sense_amp_price, "https://www.lcsc.com/product-detail/C2058784.html"),
            BomItem("tile", "Current comparator", "MSKSEMI LM393 dual comparator (LCSC C5252905)", tile_current_comparators, Fixed.current_comparator_price, "https://www.lcsc.com/product-detail/C5252905.html"),
            BomItem("tile", "Current front-end passives", "Sense filters and dividers", tile_current_frontend_passives, Fixed.current_frontend_passive_price),
            BomItem("tile", "Driver gate passives", "Gate pull resistors", tile_driver_passives, 0.0010),
            BomItem("tile", "Driver decoupling", "100nF logic bypass capacitors", tile_driver_decoupling, 0.0030),
            BomItem("tile", "Driver SMT assembly", "JLCPCB automated assembly joints", tile_driver_solder_joints, Fixed.smt_assembly_cost_per_joint, "https://jlcpcb.com/help/article/pcb-assembly-faqs"),
            BomItem("tile", "Magnet wire", "UEW 0.04mm Cu (kg share)", tile_wire_kg, 18.74, "https://www.alibaba.com/product-detail/Different-Color-Enmalled-Ultra-Thin-Copper_60735084062.html"),
            BomItem("tile", "Hall position sensor", "Diodes AH49ENTR-G1 linear Hall (LCSC C314698)", tile_hall_sensors, Fixed.hall_sensor_price, "https://www.lcsc.com/product-detail/C314698.html"),
            BomItem("tile", "Hall readout mux", "TI CD74HC4067SM96 16ch analog mux (LCSC C98457)", tile_hall_muxes, Fixed.hall_sensor_mux_price, "https://www.lcsc.com/product-detail/C98457.html"),
            BomItem("tile", "Tile PCB", "4-layer FR4 10x10cm", tile_pcb_area_cm2, 0.02),
            BomItem("tile", "Tile control MCU", "STM32G431KBT6 32-pin", 1, 3.13, "https://www.digikey.com/en/products/detail/stmicroelectronics/STM32G431KBT6/10231564"),
            BomItem("tile", "Backplane connector", "B2B header, tile->mainboard", 1, 0.45),
        ]
        self.piece_items = [
            BomItem("piece", "NdFeB magnet block", "N52 4mm cube", halbach.blocks_per_platform, 0.0375, "https://www.alibaba.com/product-detail/Customized-Rare-Earth-Neodymium-Magnets-N52_1601519228921.html"),
            BomItem("piece", "Piece plastic / misc", "3D print PLA + connectors", 1, 1.4),
        ]
        psu_part, _psu_rating, psu_price, psu_url = Fixed.psu_options[config.bus_voltage]
        self.board_items = [
            BomItem("board", "Compute module", "RPi CM5 2GB Lite, SC1556 (57.37 EUR)", 1, 61.96, "https://www.digikey.com/en/products/detail/raspberry-pi/SC1556/25805567"),
            BomItem("board", "Mainboard", "Custom 4-layer carrier (CM5 + power + tile links)", 1, 25.0),
            BomItem("board", "Tile interconnect", "B2B header, mainboard side", tiles.tile_count, 0.45),
            BomItem("board", "Bus power supply", psu_part, 1, psu_price, psu_url),
        ]
        if thermal.fan_count:
            self.board_items.append(BomItem(
                "board",
                "Radiator fan",
                "Noctua NF-A20 PWM 200mm at 550rpm low-noise setting",
                thermal.fan_count,
                Fixed.cooling_fan_price,
                Fixed.cooling_fan_url,
            ))

        self.per_tile_cost = sum(i.line_cost for i in self.tile_items)
        self.per_piece_cost = sum(i.line_cost for i in self.piece_items)
        self.board_shared_cost = sum(i.line_cost for i in self.board_items)
        self.tiles_cost = self.per_tile_cost * self.tile_count
        self.pieces_cost = self.per_piece_cost * self.piece_count
        self.total = self.tiles_cost + self.pieces_cost + self.board_shared_cost


board = BoardGeometry()
coil = CoilBed(board)
sim = levitation_sim.measure(levitation_sim.SimGeometry(
    magnet_cube_edge_mm=Inputs.magnet_cube_edge,
    magnets_per_period=Inputs.magnets_per_period,
    periods_per_side=Inputs.periods_per_side,
    magnet_to_coil_distance_mm=Inputs.magnet_to_coil_distance,
    plastic_wall_thickness_mm=Inputs.plastic_wall_thickness,
    base_corner_standoff_mm=Fixed.base_corner_standoff,
    square_fill_ratio=Fixed.square_fill_ratio,
    remanence=Constants.ndfeb_remanence_br,
    ndfeb_density_g_per_mm3=Constants.ndfeb_density,
    plastic_density_g_per_mm3=Constants.plastic_density,
    gravity=Constants.gravity,
    reference_king_height_mm=Fixed.reference_king_height,
    reference_king_base_diameter_mm=Fixed.reference_king_base_diameter,
    com_height_fraction=Fixed.com_height_fraction,
    coil_outer_width_mm=coil.outer_width,
    coil_outer_length_mm=coil.outer_length,
    coil_radial_width_mm=coil.winding_radial_width,
    coil_height_mm=Fixed.nominal_coil_height_for_field,
    control_cells_per_side=coil.control_cells_per_side,
    hall_sensor_pitch_mm=Fixed.hall_sensor_pitch,
    hall_observation_window_side=Fixed.hall_observation_window_side,
    driver_channel_current=Fixed.driver_channel_current,
    control_loop_bandwidth_margin=Inputs.control_loop_bandwidth_margin,
    target_tilt_deg=Inputs.target_tilt_angle_deg,
))
halbach = HalbachArray(board, sim)
piece = Piece(board, halbach)
snap = NeighbourSnap(board, piece, halbach, sim)
sweep = ConfigurationSweep(board, coil, halbach, piece, sim)
config = sweep.selected
coil.outer_height = config.coil_height
wire = WireThermal(coil, config)
driver = DiscreteDriver(board, coil, config)
passive_thermal = RadiatorCooling(board, config, driver, 0)
thermal = RadiatorCooling(board, config, driver, Inputs.active_cooling_fans)
propulsion = Propulsion(board, coil, piece, config, halbach, sim)
attitude = AttitudeAuthority(board, piece, config, sim)
control = Control(coil, config, sim)
tiles = TileControl(board, coil, control)
drive = DriveMatrix(coil, control, driver, tiles)
sensing = HallSensing(board, control, tiles, sim)
psu = PowerSupply(coil, wire, tiles, config, driver, thermal)
stability = Stability(board, piece, control, sim)
checks = StatusChecks(board, coil, halbach, piece, snap, config, control, sensing, thermal, propulsion, attitude, stability, drive, tiles, psu, sim)
bom = BillOfMaterials(board, coil, halbach, wire, config, tiles, sensing, thermal)
mass = MassBudget(board, wire, piece, thermal)


def format_value(value):
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    if value != 0 and (abs(value) < 1e-3 or abs(value) >= 1e6):
        return f"{value:.4g}"
    text = f"{value:.4f}".rstrip("0").rstrip(".")
    return text if text else "0"


def print_section(title, cells):
    print()
    print(title)
    print("-" * len(title))
    for cell in cells:
        suffix = f" {cell.unit}" if cell.unit else ""
        print(f"  {cell.label:<42}{format_value(cell.value)}{suffix}")


def print_sweep(sweep):
    print()
    title = "Configuration sweep (lightest feasible coil per bus voltage)"
    print(title)
    print("-" * len(title))
    print(f"  {'bus V':>6}{'wire':>16}{'layers':>7}{'turns':>7}{'op mA':>8}{'source C':>9}{'avail x':>9}")
    for entry in sweep.best_per_voltage:
        marker = "  <- selected" if entry is sweep.selected else ""
        entry_driver = DiscreteDriver(board, coil, entry)
        source_temp = RadiatorCooling(board, entry, entry_driver, Inputs.active_cooling_fans).cyclic_peak_source_temp
        print(f"  {entry.bus_voltage:>6}{entry.wire.label:>16}{entry.layers:>7}{entry.turns:>7}{entry.operating_current*1000:>8.1f}{source_temp:>9.1f}{entry.available_margin:>9.2f}{marker}")


def print_bom_group(items):
    for item in items:
        link = f"  {item.link}" if item.link else ""
        print(f"  {item.category:<21}{item.spec:<46}qty {format_value(item.qty_per_unit):>8}  ${item.unit_cost:>7.3f}  ${item.line_cost:>9.2f}{link}")


def print_bom(bill):
    print()
    title = "BOM (per board; listed volume pricing, 100-board order)"
    print(title)
    print("-" * len(title))

    print()
    print("  [A] Per tile (one 10x10cm coil-array module)")
    print_bom_group(bill.tile_items)
    print(f"  {'PER-TILE SUBTOTAL':<21}{'':<46}{'':>12}  {'':>8}  ${bill.per_tile_cost:>9.2f}")

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
    print(f"  {'Tiles':<21}{f'{bill.tile_count} x ${bill.per_tile_cost:.2f}':<46}{'':>12}  {'':>8}  ${bill.tiles_cost:>9.2f}")
    print(f"  {'Pieces':<21}{f'{bill.piece_count} x ${bill.per_piece_cost:.2f}':<46}{'':>12}  {'':>8}  ${bill.pieces_cost:>9.2f}")
    print(f"  {'Board-shared':<21}{'':<46}{'':>12}  {'':>8}  ${bill.board_shared_cost:>9.2f}")
    print(f"  {'BOARD TOTAL':<21}{'':<46}{'':>12}  {'':>8}  ${bill.total:>9.2f}")


def print_report():
    print("LEVITATING CHESS COIL-DENSITY MODEL")
    print_section("Platform and board geometry", board.cells())
    print_section("Coil geometry", coil.cells())
    print_section("Magnet array", halbach.cells())
    print_section("Piece mass", piece.cells())
    print_section("Neighbour snap", snap.cells())
    print_section("Selected coil configuration", config.cells())
    print_sweep(sweep)
    print_section("Wire and thermal", wire.cells())
    print_section("Discrete coil drivers", driver.cells())
    print_section("Passive baseplate and radiator cooling", passive_thermal.cells())
    print_section("Low-noise active radiator cooling", thermal.cells())
    print_section("Propulsion / flight", propulsion.cells())
    print_section("Attitude authority (tilt / yaw)", attitude.cells())
    print_section("Control feasibility", control.cells())
    print_section("Drive matrix (position-addressed)", drive.cells())
    print_section("Hall position sensing", sensing.cells())
    print_section("Tiled control architecture", tiles.cells())
    print_section("Power supply", psu.cells())
    print_section("Stability and vibration", stability.cells())
    print_section("Mass budget (whole set)", mass.cells())
    print_section("Status checks", checks.cells())
    print_bom(bom)


def run_report():
    output = StringIO()
    with redirect_stdout(output):
        print_report()
    report = output.getvalue()
    Path(__file__).with_name("last_run.txt").write_text(report)
    print(report, end="")


if __name__ == "__main__":
    run_report()
