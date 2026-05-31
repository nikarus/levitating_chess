from math import pi, sqrt, exp, ceil, floor


class Inputs:
    magnet_cube_edge = 4
    magnets_per_period = 4
    periods_per_side = 2
    magnet_to_coil_distance = 3
    plastic_wall_thickness = 1.5
    coil_winding_height = 3.5
    move_pulse_duration = 10
    max_hover_duration = 2
    spot_cooldown_duration = 60
    allowed_wire_temp_rise = 40
    force_safety_factor = 1.3
    min_maneuver_accel_g = 0.3
    position_sense_resolution_um = 5
    ambient_temperature = 35
    max_surface_temperature = 50
    control_loop_bandwidth_margin = 5
    pieces_levitating_simultaneously = 32
    sense_look_ahead_factor = 1.5
    drive_look_ahead_factor = 1.5
    production_volume = 100


class Fixed:
    base_material_clearance = 1
    square_fill_ratio = 0.8
    captured_pieces_total = 32
    captured_side_areas = 2
    captured_packing_efficiency = 0.9
    herringbone_orientation_families = 2
    halbach_first_harmonic_coefficient = 0.65
    reference_king_height = 95
    reference_king_base_diameter = 44
    wire_enamel_outside_factor = 1.08
    coil_aspect_ratio_target = 2.5
    winding_radial_width_factor = 0.35
    force_straight_length_efficiency = 0.65
    surface_heat_transfer_coefficient = 12
    heat_spread_area_factor = 1
    sink_channels_per_chip = 24
    driver_output_voltage_rating = 30
    driver_channel_current = 0.03
    usable_bus_voltage_fraction = 0.9
    ldc_sample_rate_per_channel = 4000
    coils_per_sense_channel = 16
    windings_per_coil_body = 2
    bifilar_wires_per_turn = 2
    matrix_switch_cost = 0.045
    pcb_thickness = 1.6
    unmodeled_mass_factor = 1.3
    control_tile_side = 100
    piece_control_flops = 20000
    node_mcu_throughput_mflops = 170


class Constants:
    board_squares_per_side = 8
    ndfeb_remanence_br = 1.45
    ndfeb_density = 0.0075
    plastic_density = 0.0012
    copper_resistivity = 1.724e-08
    copper_density = 8960
    copper_heat_capacity = 385
    standard_wire_diameters = [0.03, 0.04, 0.05, 0.063, 0.071, 0.08, 0.09, 0.1, 0.112, 0.125, 0.14, 0.16]
    standard_bus_voltages = [5, 9, 12, 15, 19, 24, 36, 48]
    gravity = 9.80665
    vacuum_permeability = 1.25663706e-6
    fr4_density = 0.00185


class Cell:
    def __init__(self, label, value, unit=""):
        self.label = label
        self.value = value
        self.unit = unit


class BoardGeometry:
    def __init__(self):
        self.period_length = Inputs.magnets_per_period * Inputs.magnet_cube_edge
        self.platform_side = Inputs.periods_per_side * self.period_length
        self.base_diameter = self.platform_side * sqrt(2) + Fixed.base_material_clearance
        self.square_size = self.base_diameter / Fixed.square_fill_ratio
        self.board_side = Constants.board_squares_per_side * self.square_size
        self.captured_per_side = ceil(Fixed.captured_pieces_total / Fixed.captured_side_areas)
        self.captured_rows = max(1, floor(self.board_side / self.base_diameter))
        self.captured_columns = ceil(self.captured_per_side / self.captured_rows)
        self.storage_width_each = self.captured_columns * self.base_diameter / Fixed.captured_packing_efficiency + self.base_diameter * 0.15
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
            Cell("Total motor width", self.motor_width, "mm"),
            Cell("Total motor height", self.motor_height, "mm"),
            Cell("Total motor area", self.motor_area, "mm2"),
        ]


class CoilBed:
    def __init__(self, board):
        self.outer_width = board.period_length / 2
        self.columns = 2 * Inputs.periods_per_side
        self.rows = max(1, round(board.platform_side / (Fixed.coil_aspect_ratio_target * self.outer_width)))
        self.outer_length = board.platform_side / self.rows
        self.outer_height = Inputs.coil_winding_height
        self.aspect_ratio = self.outer_length / self.outer_width
        self.bodies_per_orientation = self.columns * self.rows
        self.bodies_under_platform = Fixed.herringbone_orientation_families * self.bodies_per_orientation
        self.footprint_area = self.outer_width * self.outer_length
        self.body_density = self.bodies_under_platform / (board.platform_side ** 2)
        self.coil_spacing = sqrt(1 / self.body_density)
        self.total_bodies = ceil(board.motor_area * self.body_density)
        self.windings = self.total_bodies * Fixed.windings_per_coil_body
        self.active_bodies = self.bodies_under_platform * Inputs.pieces_levitating_simultaneously
        self.active_windings = self.active_bodies * Fixed.windings_per_coil_body
        self.peak_driven_windings = ceil(self.active_windings * Inputs.drive_look_ahead_factor)
        self.chips = ceil(self.peak_driven_windings / Fixed.sink_channels_per_chip)

    def cells(self):
        return [
            Cell("Coil outer width", self.outer_width, "mm"),
            Cell("Coil outer length", self.outer_length, "mm"),
            Cell("Coil outer height", self.outer_height, "mm"),
            Cell("Coil aspect ratio (actual)", self.aspect_ratio),
            Cell("Coil body footprint area", self.footprint_area, "mm2"),
            Cell("Coil columns across width", self.columns),
            Cell("Coil rows along length", self.rows),
            Cell("Coil bodies per orientation", self.bodies_per_orientation),
            Cell("Coil bodies under platform", self.bodies_under_platform),
            Cell("Coil body density", self.body_density, "1/mm2"),
            Cell("Equivalent coil spacing", self.coil_spacing, "mm"),
            Cell("Total bifilar coil bodies", self.total_bodies),
            Cell("Total windings (full board)", self.windings),
            Cell("Active bodies (all pieces at once)", self.active_bodies),
            Cell("Lift windings (pieces at once)", self.active_windings),
            Cell("Peak driven windings (+thrust look-ahead)", self.peak_driven_windings),
            Cell("24-channel driver chips (active)", self.chips),
        ]


class HalbachArray:
    def __init__(self, board):
        self.blocks_per_side = Inputs.periods_per_side * Inputs.magnets_per_period
        self.block_volume = Inputs.magnet_cube_edge ** 3
        self.blocks_per_platform = self.blocks_per_side ** 2
        self.block_mass = self.block_volume * Constants.ndfeb_density
        self.magnet_mass = self.blocks_per_platform * self.block_mass
        self.decay_constant = sqrt(2) * 2 * pi / board.period_length
        self.thickness_factor = 1 - exp(-self.decay_constant * Inputs.magnet_cube_edge)
        self.decay_factor = exp(-self.decay_constant * Inputs.magnet_to_coil_distance)
        self.b_at_coils = Constants.ndfeb_remanence_br * Fixed.halbach_first_harmonic_coefficient * self.thickness_factor * self.decay_factor

    def cells(self):
        return [
            Cell("Magnet blocks per side", self.blocks_per_side),
            Cell("Magnet block volume", self.block_volume, "mm3"),
            Cell("Magnet blocks per platform", self.blocks_per_platform),
            Cell("Magnet block mass", self.block_mass, "g"),
            Cell("Magnet mass total", self.magnet_mass, "g"),
            Cell("Halbach decay constant (2-D)", self.decay_constant, "1/mm"),
            Cell("Magnet thickness factor", self.thickness_factor),
            Cell("Distance decay factor", self.decay_factor),
            Cell("Estimated effective B at coils", self.b_at_coils, "T"),
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


class CoilConfiguration:
    def field_averaging_factor(self, decay_constant, stack_height):
        decay_over_height = decay_constant * stack_height
        return (1 - exp(-decay_over_height)) / decay_over_height

    def __init__(self, coil, halbach, piece, wire_diameter, bus_voltage):
        self.wire_diameter = wire_diameter
        self.bus_voltage = bus_voltage
        self.outside_diameter = wire_diameter * Fixed.wire_enamel_outside_factor
        self.radial_width = coil.outer_width * Fixed.winding_radial_width_factor
        self.turns_per_layer = max(1, floor(self.radial_width / (Fixed.bifilar_wires_per_turn * self.outside_diameter)))
        self.layers = max(1, floor(Inputs.coil_winding_height / self.outside_diameter))
        self.turns = self.turns_per_layer * self.layers
        self.coil_height = self.layers * self.outside_diameter
        self.straight_length = 2 * (coil.outer_length - self.radial_width) * Fixed.force_straight_length_efficiency
        self.average_length_per_turn = 2 * ((coil.outer_length - self.radial_width) + (coil.outer_width - self.radial_width))
        self.length_per_winding = self.turns * self.average_length_per_turn / 1000
        self.cross_section_area = pi * (wire_diameter / 1000) ** 2 / 4
        self.resistance = Constants.copper_resistivity * self.length_per_winding / self.cross_section_area
        self.voltage_limited_current = bus_voltage * Fixed.usable_bus_voltage_fraction / self.resistance
        self.thermal_limited_current = self.cross_section_area * sqrt(Inputs.allowed_wire_temp_rise * Constants.copper_density * Constants.copper_heat_capacity / (Constants.copper_resistivity * Inputs.move_pulse_duration))
        self.current_limit = min(self.voltage_limited_current, self.thermal_limited_current)
        self.averaging_factor = self.field_averaging_factor(halbach.decay_constant, self.coil_height)
        self.average_b = halbach.b_at_coils * self.averaging_factor
        self.force_per_amp = self.turns * self.average_b * (self.straight_length / 1000) * coil.bodies_under_platform
        self.available_force = self.current_limit * self.force_per_amp
        self.available_margin = self.available_force / piece.weight
        self.required_current = piece.weight * Inputs.force_safety_factor / self.force_per_amp
        self.operating_current = self.required_current
        self.force_per_body = self.operating_current * self.turns * self.average_b * (self.straight_length / 1000)
        self.total_force = self.force_per_body * coil.bodies_under_platform
        self.margin = self.total_force / piece.weight
        self.voltage_per_winding = self.operating_current * self.resistance
        self.power_per_winding = self.operating_current ** 2 * self.resistance
        self.temp_rise = self.operating_current ** 2 * self.resistance * Inputs.move_pulse_duration / (Constants.copper_density * self.length_per_winding * self.cross_section_area * Constants.copper_heat_capacity)
        self.surface_temperature = Inputs.ambient_temperature + self.temp_rise

    def cells(self):
        return [
            Cell("Selected standard wire dia", self.wire_diameter, "mm"),
            Cell("Wire outside diameter", self.outside_diameter, "mm"),
            Cell("Selected bus voltage", self.bus_voltage, "V"),
            Cell("Winding radial width", self.radial_width, "mm"),
            Cell("Turns per radial layer", self.turns_per_layer),
            Cell("Vertical wire layers", self.layers),
            Cell("Turns per winding (fits window)", self.turns),
            Cell("Coil height", self.coil_height, "mm"),
            Cell("Useful straight length per turn", self.straight_length, "mm"),
            Cell("Wire length per winding", self.length_per_winding, "m"),
            Cell("Resistance per winding", self.resistance, "ohm"),
            Cell("Voltage-limited current", self.voltage_limited_current, "A"),
            Cell("Thermal-limited current", self.thermal_limited_current, "A"),
            Cell("Wire current limit", self.current_limit, "A"),
            Cell("Field averaging factor over coil", self.averaging_factor),
            Cell("Average B over coil height", self.average_b, "T"),
            Cell("Available lift force (at limit)", self.available_force, "N"),
            Cell("Available lift margin", self.available_margin, "x"),
            Cell("Operating current per winding", self.operating_current, "A"),
            Cell("Lift force per coil body", self.force_per_body, "N"),
            Cell("Total lift force", self.total_force, "N"),
            Cell("Lift margin (with safety)", self.margin, "x"),
            Cell("Pulse temp rise at operating current", self.temp_rise, "C"),
            Cell("Surface temp after one pulse", self.surface_temperature, "C"),
        ]


class ConfigurationSweep:
    def __init__(self, coil, halbach, piece):
        self.configurations = [
            CoilConfiguration(coil, halbach, piece, wire_diameter, bus_voltage)
            for bus_voltage in Constants.standard_bus_voltages
            for wire_diameter in Constants.standard_wire_diameters
        ]
        self.feasible = [
            c for c in self.configurations
            if c.available_margin >= Inputs.force_safety_factor
            and c.bus_voltage <= Fixed.driver_output_voltage_rating
            and c.current_limit <= Fixed.driver_channel_current
        ]
        self.selected = min(self.feasible, key=lambda c: c.power_per_winding)
        self.best_per_voltage = []
        for bus_voltage in Constants.standard_bus_voltages:
            candidates = [c for c in self.feasible if c.bus_voltage == bus_voltage]
            if candidates:
                self.best_per_voltage.append(min(candidates, key=lambda c: c.power_per_winding))


class WireThermal:
    def __init__(self, coil, config):
        self.mass_per_winding = Constants.copper_density * config.length_per_winding * config.cross_section_area * 1000
        self.wire_length = config.length_per_winding * coil.windings
        self.copper_mass = self.mass_per_winding * coil.windings / 1000
        self.one_piece_power = config.power_per_winding * coil.bodies_under_platform
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


class SurfaceThermal:
    def __init__(self, board, coil, config):
        self.active_copper_mass = Constants.copper_density * config.length_per_winding * config.cross_section_area * coil.bodies_under_platform
        self.thermal_capacitance = self.active_copper_mass * Constants.copper_heat_capacity
        self.dissipation_area = board.platform_side ** 2 * Fixed.heat_spread_area_factor / 1000000
        self.thermal_conductance = Fixed.surface_heat_transfer_coefficient * self.dissipation_area
        self.thermal_time_constant = self.thermal_capacitance / self.thermal_conductance
        self.one_piece_power = config.power_per_winding * coil.bodies_under_platform
        self.pulse_surface_temp = Inputs.ambient_temperature + config.temp_rise
        self.max_pulse_duration = self.thermal_capacitance * (Inputs.max_surface_temperature - Inputs.ambient_temperature) / self.one_piece_power
        self.spot_period = Inputs.max_hover_duration + Inputs.spot_cooldown_duration
        self.spot_duty_cycle = Inputs.max_hover_duration / self.spot_period
        self.duty_average_power = self.one_piece_power * self.spot_duty_cycle
        self.steady_state_rise = self.duty_average_power / self.thermal_conductance
        self.steady_state_surface_temp = Inputs.ambient_temperature + self.steady_state_rise
        self.full_hover_rise = self.one_piece_power / self.thermal_conductance
        self.cyclic_peak_rise = self.full_hover_rise * (1 - exp(-Inputs.max_hover_duration / self.thermal_time_constant)) / (1 - exp(-self.spot_period / self.thermal_time_constant))
        self.cyclic_peak_surface_temp = Inputs.ambient_temperature + self.cyclic_peak_rise

    def cells(self):
        return [
            Cell("Active copper mass (one piece)", self.active_copper_mass * 1000, "g"),
            Cell("Lumped thermal capacitance", self.thermal_capacitance, "J/K"),
            Cell("Heat-spread dissipation area", self.dissipation_area * 10000, "cm2"),
            Cell("Thermal conductance to ambient", self.thermal_conductance, "W/K"),
            Cell("Thermal time constant", self.thermal_time_constant, "s"),
            Cell("Max pulse duration to cap", self.max_pulse_duration, "s"),
            Cell("Per-spot duty cycle (2s/60s)", self.spot_duty_cycle),
            Cell("Duty-cycle average power", self.duty_average_power, "W"),
            Cell("Steady-state surface temp at duty", self.steady_state_surface_temp, "C"),
            Cell("Cyclic peak surface temp (2s hover/60s)", self.cyclic_peak_surface_temp, "C"),
        ]


class Propulsion:
    def __init__(self, board, coil, piece, config, halbach):
        self.mass = piece.mass / 1000
        self.weight = piece.weight
        self.available_force = config.available_force
        self.lateral_force_per_amp = config.force_per_amp
        self.max_thrust = sqrt(max(self.available_force ** 2 - self.weight ** 2, 0))
        self.max_acceleration = self.max_thrust / self.mass
        self.acceleration_in_g = self.max_acceleration / Constants.gravity
        self.thrust_current = self.max_thrust / self.lateral_force_per_amp
        self.flight_power = config.current_limit ** 2 * config.resistance * coil.bodies_under_platform
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
            Cell("Thrust current per winding", self.thrust_current, "A"),
            Cell("In-flight coil power (one piece)", self.flight_power, "W"),
            Cell("One-square hop time (bang-bang)", self.hop_time, "s"),
            Cell("One-square peak speed", self.hop_peak_speed, "m/s"),
            Cell("Full-rank traverse time", self.traverse_time, "s"),
            Cell("Yaw moment of inertia", self.yaw_inertia, "kg.m2"),
            Cell("Max yaw torque", self.yaw_torque, "N.m"),
            Cell("Max yaw angular acceleration", self.yaw_acceleration, "rad/s2"),
        ]


class Control:
    def coil_inductance(self, turns, footprint_area, height):
        return Constants.vacuum_permeability * turns ** 2 * (footprint_area / 1000000) / (height / 1000) * 1000

    def __init__(self, coil, config, halbach):
        self.inductance = self.coil_inductance(config.turns, coil.footprint_area, config.coil_height)
        self.time_constant = (self.inductance / 1000) / config.resistance * 1000
        self.actuator_bandwidth = 1 / (2 * pi * (self.time_constant / 1000))
        self.slew_time = (self.inductance / 1000) * config.operating_current / (config.bus_voltage * Fixed.usable_bus_voltage_fraction) * 1000
        self.instability_time = 1 / sqrt(halbach.decay_constant * 1000 * Constants.gravity) * 1000
        self.required_bandwidth = Inputs.control_loop_bandwidth_margin / (2 * pi * (self.instability_time / 1000))
        self.pose_update_rate = Inputs.control_loop_bandwidth_margin / (self.instability_time / 1000)

    def cells(self):
        return [
            Cell("Coil inductance (estimate)", self.inductance, "mH"),
            Cell("Electrical time constant L/R", self.time_constant, "ms"),
            Cell("Actuator current-loop bandwidth", self.actuator_bandwidth, "Hz"),
            Cell("Current slew time to Imax", self.slew_time, "ms"),
            Cell("Open-loop instability growth time", self.instability_time, "ms"),
            Cell("Required control loop bandwidth", self.required_bandwidth, "Hz"),
            Cell("Required pose update rate", self.pose_update_rate, "Hz"),
        ]


class DriveMatrix:
    def __init__(self, coil, control, config):
        self.scheme = "direct active-select (continuous hold)"
        self.coil_switches = coil.total_bodies
        self.active_windings = coil.peak_driven_windings
        self.driver_channels = coil.chips * Fixed.sink_channels_per_chip
        self.pool_headroom = self.driver_channels / self.active_windings
        self.slew_time = control.slew_time
        self.update_period = 1000 / control.pose_update_rate
        self.slew_over_update = self.slew_time / self.update_period

    def cells(self):
        return [
            Cell("Drive scheme", self.scheme),
            Cell("Per-coil select switches", self.coil_switches),
            Cell("Peak windings driven at once", self.active_windings),
            Cell("Driver output channels (pool)", self.driver_channels),
            Cell("Driver pool headroom", self.pool_headroom, "x"),
            Cell("Current slew time to Imax", self.slew_time, "ms"),
            Cell("Control update period", self.update_period, "ms"),
            Cell("Slew / update-period ratio", self.slew_over_update, "x"),
        ]


class Sensing:
    def __init__(self, coil, control):
        self.per_coil_update_rate = control.pose_update_rate
        self.active_coils = Inputs.pieces_levitating_simultaneously * coil.bodies_under_platform * Inputs.sense_look_ahead_factor
        self.demand = self.active_coils * self.per_coil_update_rate
        self.channels = ceil(coil.total_bodies / (4 * Fixed.coils_per_sense_channel)) * 4
        self.capacity = self.channels * Fixed.ldc_sample_rate_per_channel
        self.headroom = self.capacity / self.demand

    def cells(self):
        return [
            Cell("Required per-coil update rate", self.per_coil_update_rate, "Hz"),
            Cell("Active coils sensed (worst case)", self.active_coils, "coils"),
            Cell("Reads needed", self.demand, "reads/s"),
            Cell("Total sense channels", self.channels),
            Cell("Total sensing capacity", self.capacity, "reads/s"),
            Cell("Sensing headroom", self.headroom, "x"),
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


class Stability:
    def __init__(self, board, piece, halbach, config, control):
        self.decay_constant_m = halbach.decay_constant * 1000
        self.mass = piece.mass / 1000
        self.height = piece.box_height / 1000
        self.half_width = board.platform_side / 2 / 1000
        self.vertical_stiffness = self.decay_constant_m * piece.weight
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

    def __init__(self, board, coil, piece, config, control, sensing, thermal, propulsion, stability, drive, tiles):
        self.force = self.passes(config.available_margin >= 1, "OK", "not enough force")
        self.safety = self.passes(config.available_margin >= Inputs.force_safety_factor, "OK", "below safety margin")
        self.voltage = self.passes(config.voltage_per_winding <= config.bus_voltage * Fixed.usable_bus_voltage_fraction, "OK", "voltage too high")
        self.wire_thermal = self.passes(config.temp_rise <= Inputs.allowed_wire_temp_rise, "OK", "wire too hot")
        self.pulse_surface = self.passes(thermal.pulse_surface_temp <= Inputs.max_surface_temperature, "OK", "pulse surface too hot")
        self.duty_surface = self.passes(thermal.steady_state_surface_temp <= Inputs.max_surface_temperature, "OK", "duty surface too hot")
        self.hover_surface = self.passes(thermal.cyclic_peak_surface_temp <= Inputs.max_surface_temperature, "OK", "cyclic hover too hot")
        self.maneuvering = self.passes(propulsion.acceleration_in_g >= Inputs.min_maneuver_accel_g, "OK", "lateral thrust too weak")
        self.rock_controllable = self.passes(stability.control_margin_over_rock >= Inputs.control_loop_bandwidth_margin, "OK", "rock mode too fast for loop")
        self.tilt_observable = self.passes(stability.tip_sense_resolution <= 0.001, "OK", "tilt sensing too coarse")
        self.driver_voltage = self.passes(config.bus_voltage <= Fixed.driver_output_voltage_rating, "OK", "bus exceeds driver Vout rating")
        self.driver_current = self.passes(config.current_limit <= Fixed.driver_channel_current, "OK", "coil current exceeds channel rating")
        self.per_orientation = self.passes(coil.bodies_per_orientation >= 6, "OK", "few coils per orientation")
        self.shell_validity = self.passes((piece.diameter - 2 * Inputs.plastic_wall_thickness) > 0, "OK", "wall too thick")
        self.coil_height = self.passes(config.coil_height <= 12, "OK", "coil too tall")
        self.platform_size = self.passes(20 <= board.platform_side <= 50, "OK", "platform out of range")
        self.magnet_fits_base = self.passes(board.platform_side <= board.base_diameter, "OK", "magnet array wider than base")
        self.control_bandwidth = self.passes(control.actuator_bandwidth >= control.required_bandwidth, "OK", "actuator bandwidth too low")
        self.current_slew = self.passes(control.slew_time <= control.instability_time / Inputs.control_loop_bandwidth_margin, "OK", "current cannot react in time")
        self.active_region_sensing = self.passes(sensing.capacity >= sensing.demand, "OK", "sensing too slow for control")
        self.driver_pool = self.passes(drive.pool_headroom >= 1, "OK", "driver pool too small for active set")
        self.drive_slew = self.passes(drive.slew_over_update <= 1, "OK", "current too slow for update period")
        self.tile_compute = self.passes(tiles.tile_headroom >= 1, "OK", "tile MCU overloaded")

    def cells(self):
        return [
            Cell("Force check", self.force),
            Cell("Safety-margin check", self.safety),
            Cell("Voltage check", self.voltage),
            Cell("Wire thermal check", self.wire_thermal),
            Cell("Pulse surface temp check", self.pulse_surface),
            Cell("Duty surface temp check", self.duty_surface),
            Cell("Cyclic-hover surface temp check", self.hover_surface),
            Cell("Maneuvering check", self.maneuvering),
            Cell("Rock-mode controllable check", self.rock_controllable),
            Cell("Tilt-observable check", self.tilt_observable),
            Cell("Driver-voltage-rating check", self.driver_voltage),
            Cell("Driver-channel-current check", self.driver_current),
            Cell("Per-orientation check", self.per_orientation),
            Cell("Shell-validity check", self.shell_validity),
            Cell("Coil-height buildable check", self.coil_height),
            Cell("Platform-size check", self.platform_size),
            Cell("Magnet-array-fits-base check", self.magnet_fits_base),
            Cell("Control-bandwidth check", self.control_bandwidth),
            Cell("Current-slew check", self.current_slew),
            Cell("Active-region sensing check", self.active_region_sensing),
            Cell("Driver-pool-size check", self.driver_pool),
            Cell("Drive-slew check", self.drive_slew),
            Cell("Per-tile-compute check", self.tile_compute),
        ]


class BomItem:
    def __init__(self, category, spec, quantity, unit_cost, link=""):
        self.category = category
        self.spec = spec
        self.per_board = quantity
        self.quantity = quantity * Inputs.production_volume
        self.unit_cost = unit_cost
        self.link = link
        self.subtotal = self.quantity * unit_cost


class MassBudget:
    def __init__(self, board, wire, piece):
        self.board_copper_mass = wire.copper_mass
        self.board_pcb_mass = board.motor_area * Fixed.pcb_thickness * Constants.fr4_density / 1000
        self.board_known_mass = self.board_copper_mass + self.board_pcb_mass
        self.board_total_mass = self.board_known_mass * Fixed.unmodeled_mass_factor
        self.piece_mass = piece.mass / 1000
        self.pieces_total = Fixed.captured_pieces_total
        self.all_pieces_mass = self.piece_mass * self.pieces_total
        self.set_total_mass = self.board_total_mass + self.all_pieces_mass

    def cells(self):
        return [
            Cell("Board copper (coils)", self.board_copper_mass, "kg"),
            Cell("Board PCB (FR4)", self.board_pcb_mass, "kg"),
            Cell("Board electronics/PSU/frame factor", Fixed.unmodeled_mass_factor, "x"),
            Cell("Board total (est.)", self.board_total_mass, "kg"),
            Cell("Mass per piece", self.piece_mass * 1000, "g"),
            Cell("Pieces total", self.pieces_total),
            Cell("All pieces mass", self.all_pieces_mass, "kg"),
            Cell("WHOLE SET mass (est.)", self.set_total_mass, "kg"),
        ]


class BillOfMaterials:
    def __init__(self, board, coil, halbach, wire, config, tiles):
        self.items = [
            BomItem("Coil driver IC", "TLC5947DAP 24ch 12b PWM 30V/30mA", coil.chips, 2.799, "https://www.digikey.com/en/products/detail/texas-instruments/TLC5947DAP/1894117"),
            BomItem("Coil select switch", "BSS138-7-F N-FET, 1/coil body", coil.total_bodies, 0.028, "https://www.digikey.com/en/products/detail/diodes-incorporated/BSS138-7-F/717723"),
            BomItem("Flyback diode", "1N4148WS-7-F", coil.windings, 0.0213, "https://www.digikey.com/en/products/detail/diodes-incorporated/1N4148WS-7-F/815127"),
            BomItem("Magnet wire", "UEW 0.04mm solid copper, bulk kg", ceil(wire.copper_mass), 18.74, "https://www.alibaba.com/product-detail/Different-Color-Enmalled-Ultra-Thin-Copper_60735084062.html"),
            BomItem("NdFeB magnet block", "N52 4mm cube", halbach.blocks_per_platform * Inputs.pieces_levitating_simultaneously, 0.0375, "https://www.alibaba.com/product-detail/Customized-Rare-Earth-Neodymium-Magnets-N52_1601519228921.html"),
            BomItem("Tile control MCU", "STM32G431KBT6 32-pin, 1/tile", tiles.tile_count, 3.13, "https://www.digikey.com/en/products/detail/stmicroelectronics/STM32G431KBT6/10231564"),
            BomItem("Host computer + UI", "SBC + touchscreen: path planner, game logic, chess.com", 1, 75.0),
            BomItem("Bus power supply", f"{config.bus_voltage}V regulated supply", 1, 30.0),
            BomItem("PCB main board", "custom 4-layer FR4", round(board.motor_area / 100), 0.02),
            BomItem("Coil-sense AFE", "LDC1614RGHR", ceil(coil.total_bodies / (4 * Fixed.coils_per_sense_channel)), 2.249, "https://www.digikey.com/en/products/detail/texas-instruments/LDC1614RGHR/5481860"),
            BomItem("Sense analog mux", "CD74HC4067M96", ceil(coil.total_bodies / Fixed.coils_per_sense_channel), 0.3405, "https://www.digikey.com/en/products/detail/texas-instruments/CD74HC4067M96/1507236"),
            BomItem("Piece ID LC tag", "LQM18FN100M00D + C0G cap", Inputs.pieces_levitating_simultaneously, 0.15, "https://www.digikey.com/en/products/detail/murata-electronics/LQM18FN100M00D/1016184"),
            BomItem("Piece plastic / misc", "3D print PLA + connectors", 1, 45.0),
        ]
        self.total = sum(item.subtotal for item in self.items)


board = BoardGeometry()
coil = CoilBed(board)
halbach = HalbachArray(board)
piece = Piece(board, halbach)
sweep = ConfigurationSweep(coil, halbach, piece)
config = sweep.selected
wire = WireThermal(coil, config)
thermal = SurfaceThermal(board, coil, config)
propulsion = Propulsion(board, coil, piece, config, halbach)
control = Control(coil, config, halbach)
drive = DriveMatrix(coil, control, config)
sensing = Sensing(coil, control)
tiles = TileControl(board, coil, control)
stability = Stability(board, piece, halbach, config, control)
checks = StatusChecks(board, coil, piece, config, control, sensing, thermal, propulsion, stability, drive, tiles)
bom = BillOfMaterials(board, coil, halbach, wire, config, tiles)
mass = MassBudget(board, wire, piece)


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
    title = "Configuration sweep (coolest feasible wire per bus voltage)"
    print(title)
    print("-" * len(title))
    print(f"  {'bus V':>6}{'wire mm':>9}{'turns':>7}{'op mA':>8}{'surf C':>8}{'avail x':>9}")
    for entry in sweep.best_per_voltage:
        marker = "  <- selected" if entry is sweep.selected else ""
        print(f"  {entry.bus_voltage:>6}{entry.wire_diameter:>9}{entry.turns:>7}{entry.operating_current*1000:>8.1f}{entry.surface_temperature:>8.1f}{entry.available_margin:>9.2f}{marker}")


def print_bom(bill):
    print()
    title = "BOM (per board; unit prices are volume pricing for a 100-board order)"
    print(title)
    print("-" * len(title))
    for item in bill.items:
        link = f"  {item.link}" if item.link else ""
        line_cost = item.per_board * item.unit_cost
        print(f"  {item.category:<22}{item.spec:<40}qty {format_value(item.per_board):>8}  ${item.unit_cost:>7.3f}  ${line_cost:>9.2f}{link}")
    print(f"  {'BOARD TOTAL':<22}{'':<40}{'':>12}  {'':>8}  ${bill.total / Inputs.production_volume:>9.2f}")


def print_report():
    print("LEVITATING CHESS COIL-DENSITY MODEL")
    print_section("Platform and board geometry", board.cells())
    print_section("Coil geometry", coil.cells())
    print_section("Magnet array", halbach.cells())
    print_section("Piece mass", piece.cells())
    print_section("Selected coil configuration", config.cells())
    print_sweep(sweep)
    print_section("Wire and thermal", wire.cells())
    print_section("Surface temperature (passive cooling)", thermal.cells())
    print_section("Propulsion / flight", propulsion.cells())
    print_section("Control feasibility", control.cells())
    print_section("Drive matrix (position-addressed)", drive.cells())
    print_section("Sensing throughput", sensing.cells())
    print_section("Tiled control architecture", tiles.cells())
    print_section("Stability and vibration", stability.cells())
    print_section("Mass budget (whole set)", mass.cells())
    print_section("Status checks", checks.cells())
    print_bom(bom)


print_report()
