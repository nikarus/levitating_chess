from contextlib import redirect_stdout
from io import StringIO
from math import pi, sqrt, exp, ceil, floor, sin, asin, radians, degrees, log, log10, log2
from pathlib import Path

import levitation_sim


class Inputs:
    magnet_lateral_edge = 5                    # mm
    magnet_thickness = 5                       # mm
    magnets_per_period = 4                     # count
    periods_per_side = 1                       # count
    coils_per_period = 2                       # count
    coil_outer_length = 15                     # mm
    winding_radial_width = 1.0                 # mm
    control_cells_per_side = 4                 # count
    magnet_to_coil_distance = 3.5              # mm
    max_flight_gap = 4.5                       # mm
    plastic_wall_thickness = 1.0               # mm
    worst_phase_dwell = 0.5                    # s (lift-off + slide-to-phase from adversarial hand placement)
    cruise_duration = 1.5                      # s (up to full-diagonal flight at phase-averaged power)
    contention_hover = 2.0                     # s (choreography wait, held at a phase-aligned lattice point)
    landing_dwell = 0.5                        # s (aligned-slot approach and settle)
    events_back_to_back = 2                    # count (immediate-rematch resets with no cooldown)
    sustained_event_period = 300               # s (one full reset event sustained forever)
    play_move_period = 6                       # s (silent mode: live human play, one move per period)
    sustained_moves_per_minute = 60            # moves/min (spectate mode: real-time fast-game replay, no lag ever)
    replay_capture_fraction = 0.25             # ratio (replay moves that add a second storage flight)
    replay_knight_gap_fraction = 0.15          # ratio (replay moves needing blockers to part and re-close)
    hammer_visit_period = 5                    # s (W3: one aligned dwell on the same cell, forever)
    hammer_dwell = 1.0                         # s (W3: takeoff or landing dwell per visit)
    cascade_exchanges = 10                     # count (W3: max capture exchanges on one square, full-speed burst)
    coil_bed_temp_limit = 105                  # C (internal: Appli-Thane-class potting rated ~121C continuous, enamel wire class 155; external surface limits govern separately)
    force_safety_factor = 1.3                  # ratio
    min_maneuver_accel_g = 0.2                 # g
    min_visible_hover_height = 3               # mm (piece bottom to playing surface at nominal hover)
    tilt_rim_clearance = 1.0                   # mm (tilt limited so the lowest rim point keeps this clearance above the playing surface)
    target_tilt_time = 0.3                     # s
    target_yaw_angle_deg = 90                  # deg
    target_yaw_time = 0.5                      # s
    ambient_temperature = 35                   # C
    max_surface_temperature = 50               # C
    control_loop_bandwidth_margin = 5          # ratio
    pieces_levitating_simultaneously = 32      # count
    drive_look_ahead_factor = 1.5              # ratio
    production_volume = 100                    # boards
    active_cooling_fans = 2                    # count (spectate mode only; silent mode keeps them off)


class Fixed:
    base_corner_standoff = 8                   # mm
    square_fill_ratio = 0.8                    # ratio (maximum; actual fill falls out of period snapping)
    max_chess_square_size = 60                 # mm
    resting_friction_coefficient = 0.4         # ratio
    captured_pieces_total = 32                 # count
    captured_side_areas = 2                    # count
    herringbone_orientation_families = 2       # count
    com_height_fraction = 0.4                  # ratio
    reference_king_height = 95                 # mm
    reference_king_base_diameter = 44          # mm
    rectangular_wire_film = 0.012              # mm per side
    turns_per_radial_layer = 1                 # count
    nominal_coil_height_for_field = 1.0        # mm
    potting_thickness = 1.0                    # mm
    potting_thermal_conductivity = 2.5         # W/(m.K) Appli-Thane 7300 class thermal potting
    potting_volumetric_heat_capacity = 2.0e6   # J/(m3.K) (epoxy/copper bed, conservative)
    coil_bed_through_conductivity = 2.0        # W/(m.K) (through-plane: enamel-film wire stack in parallel with potted coil windows)
    potting_cover_thickness = 0.2              # mm (self-leveling skim over coil tops)
    playing_surface_thickness = 0.1            # mm (UV-printed graphics + clear wear topcoat directly on the potting; no separate sheet)
    playing_surface_conductivity = 0.2         # W/(m.K) epoxy topcoat
    piece_bottom_skin = 0.1                    # mm (plated magnets flush, thin conformal coat)
    surface_flatness_budget = 0.3              # mm (over full board span)
    max_touch_temperature = 77                 # C (IEC 62368-1 plastic 1-10s brief contact; hotspot is covered by the hovering piece, touchable only during post-departure cooldown)
    prolonged_touch_temperature = 48           # C (IEC 62368-1 prolonged-contact plastic; idle regions)
    magnet_max_operating_temperature = 150     # C (SH-grade NdFeB rating; resting piece soaks toward the cell temperature)
    pcb_via_effective_thermal_conductivity = 10 # W/(m.K) (dense 0.3mm thermal-via array under the coil bed, ~1mm pitch)
    thermal_pad_conductivity = 5.0             # W/(m.K) premium dispensed gap filler
    baseplate_thickness = 4.0                  # mm
    aluminium_thermal_conductivity = 167       # W/(m.K)
    aluminium_density = 2700                   # kg/m3
    aluminium_heat_capacity = 900              # J/(kg.K)
    fin_height = 30                            # mm (sized for fanless dissipation at 3.5mm hover gap)
    fin_thickness = 2                          # mm
    fin_channel_width = 8                      # mm
    natural_convection_coefficient = 2         # W/(m2.K) (derated: fins face down under an elevated board)
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
    mosfet_voltage_rating = 40                 # V (right-sized: 24V split rail + switching margin)
    driver_channel_current = 5.5               # A
    driver_pwm_frequency = 20000               # Hz
    driver_switching_time = 100e-9             # s
    driver_hot_resistance = 0.08               # ohm
    driver_mosfet_gate_charge = 30e-9          # C
    gate_drive_voltage = 10                    # V
    logic_gate_voltage = 5                     # V
    current_command_rate = 500                 # Hz
    setpoint_filter_passive_price = 0.0030     # USD per 15.8k + 10nF pair (catalog target; production RFQ needed)
    setpoint_filter_cutoff = 1000              # Hz
    max_setpoint_bits = 12                     # count
    max_current_command_error = 0.005          # A
    current_loop_bandwidth = 2000              # Hz
    min_current_loop_pwm_cycles = 8            # count
    current_monitor_adc_sample_rate = 500000   # samples/s
    current_sense_resistance = 0.02            # ohm
    comparator_input_offset = 0.005            # V (LM393 raw, removed by idle zero-calibration)
    current_sense_offset_residual = 0.001      # V (post-calibration drift: comparator + level-shift divider tempco over the operating range)
    max_current_offset_fraction = 0.05         # ratio of driver limit
    current_shunt_price = 0.0418               # USD (Milliohm HoJLR2512-2W-20mR-1% 75ppm, LCSC C2924538 at 4k; full-reel RFQ pending)
    current_shunt_power_rating = 2             # W
    current_comparator_channels_per_ic = 2     # count
    current_comparator_price = 0.0198          # USD
    current_frontend_passives_per_channel = 4  # count (RC filter + midpoint level-shift pair; 1% + matched arrays suffice - idle zero-cal removes statics)
    current_frontend_passive_price = 0.0035    # USD (1% thick-film + matched-pair arrays where tracking matters)
    driver_gate_passive_price = 0.0012         # USD
    driver_decoupling_price = 0.0197           # USD
    shift_register_outputs = 8                 # count
    gate_driver_half_bridges = 3               # count
    gate_driver_price = 0.2254                 # USD
    power_mosfet_price = 0.055                 # USD RFQ target (40V dual N-MOSFET SOP-8/PDFN; see BOM_SOURCING.md)
    shift_register_clock_rating = 25000000     # bits/s
    shift_register_power_capacitance = 42e-12   # F
    shift_register_price = 0.0280              # USD
    driver_serial_clock = 40000000             # bits/s
    smt_assembly_cost_per_joint = 0.0017       # USD
    max_bus_current = 120                      # A
    bus_distribution_price = 36.96             # USD
    rail_clamp_price = 5.0                     # USD (needs active MOSFET dump clamp design: 12V TVS knees near 20V vs 40V FET rating) [TO BE SOURCED]
    rail_bulk_capacitor_price = 4.0            # USD (zone-level electrolytic; sizing needs ripple/inductance spec) [TO BE SOURCED]
    tile_bulk_capacitors_per_tile = 4          # count (local power decoupling for 116 half-bridges at 20kHz; zone caps are electrically too far)
    tile_bulk_capacitor_price = 0.25           # USD (330-470uF 16V polymer class) [TO BE SOURCED]
    supercap_cell_capacitance = 350            # F (Maxwell BCAP0350 P270 S18 snap-in)
    supercap_cell_max_voltage = 2.7            # V
    supercap_cell_working_voltage = 2.4        # V (series-count derating for cell life)
    supercap_cell_esr = 0.0032                 # ohm
    supercap_cell_price = 7.65                 # USD (Maxwell BCAP0350-P270-S18, DigiKey 1k tier; allocation advised)
    supercap_cell_mass_kg = 0.065              # kg (distributor-listed)
    supercap_balancer_price_per_cell = 0.15    # USD (balancing network share) [TO BE SOURCED]
    supercap_oring_price = 4.0                 # USD (LM74800-Q1 controller + paralleled 40V N-FET pairs per rail per zone; ~103A path needs thermal validation) [TO BE SOURCED]
    supercap_management_price_per_rail = 20.0  # USD (BQ33100-class monitor + precharge/charge path + fuse + disconnect per rail bank) [TO BE SOURCED]
    bus_droop_fraction = 0.1                   # ratio (allowed rail sag during buffer-fed burst; half depletion, half ESR)
    radiator_aluminium_price_per_kg = 12.0     # USD (extruded integral-fin heatsink, RFQ budget)
    playing_surface_price = 12.0               # USD
    tile_pcb_price_per_cm2 = 0.01063           # USD
    tile_connector_header_price = 0.0724       # USD
    tile_connector_socket_price = 0.1125       # USD
    usable_bus_voltage_fraction = 0.9          # ratio
    hall_sensor_pitch = 12.5                   # mm (densest candidate; sweep-gate default)
    hall_pitch_candidates = (25.0, 20.0, 100 / 6, 100 / 7, 12.5)  # mm, sparse->dense, all divide the tile
    hall_observation_window_side = 4           # count
    hall_sensor_mux_channels = 16              # count
    hall_sensor_price = 0.34758                # USD (TI DRV5055A4QDBZR catalog tier; production RFQ needed)
    hall_sensor_mux_price = 0.2527             # USD
    magnet_cost_per_kg = 384.0                 # USD/kg equivalent of $0.36 per 5mm N48SH cube [TO BE SOURCED]
    hall_adc_sample_rate = 500000              # samples/s
    hall_adc_native_bits = 12                  # bits (MCU SAR ADC native resolution)
    hall_interpolation_bits = 14               # bits (effective, via oversampling above native)
    hall_package_standoff = 0.6                # mm (SOT-23 sensing element below PCB underside)
    hall_power_settle_time = 0.0001            # s (gated sensor group power-up + RC settle before burst)
    hall_gate_switch_price = 0.0206            # USD (GOODWORK AO3401A catalog tier; production RFQ needed)
    hall_supply_voltage = 5                    # V
    hall_supply_current = 0.005                # A max (TI DRV5055A4 at 5V)
    hall_sensitivity = 12.5                    # V/T (TI DRV5055A4 nominal at 5V)
    hall_output_noise = 0.0000184              # T rms typical (130nT/sqrtHz * sqrt(20kHz)); not a guaranteed maximum
    hall_linear_range = 0.169                  # T (TI DRV5055A4 at 5V)
    hall_sensor_bandwidth = 20000              # Hz (DRV5055 small-signal bandwidth; bounds independent noise samples)
    coil_field_subtraction_error = 0.005       # ratio (residual after Hall-array self-calibration of coil map + channel offsets, idle-board recalibration)
    position_error_gap_fraction = 0.1          # ratio (position error budget as fraction of flight gap)
    pcb_copper_plane_thickness = 0.07          # mm (two 1oz solid planes in the 4-layer tile PCB)
    radiator_standoff_below_pcb = 1.5          # mm (gap-filler pad thickness; clears SOT-23 Hall bodies under the PCB)
    radiator_slot_pitch = 5                    # mm (crosshatch eddy-break kerfs, island size)
    radiator_slot_web_thickness = 0.5          # mm (solid web left at plate bottom under the kerfs)
    radiator_slotting_price = 200.0            # USD (gang-saw crosshatch kerf pass on extrusion base, mid RFQ budget)
    potting_epoxy_price = 45.0                 # USD (alumina-filled 2.5 W/mK, coil bed allowance, RFQ budget)
    gap_filler_pad_price = 336.43              # USD (Laird Tputty SF560 5.6 W/mK, 10-pail public price per board; RFQ + selective dispensing could cut 67-80%)
    gap_filler_density = 3.40                  # g/cc (Tputty SF560)
    windings_per_coil_body = 1                 # count
    pcb_thickness = 1.6                        # mm
    frame_enclosure_mass_kg = 1.0              # kg
    board_electronics_mass_kg = 0.3            # kg
    blocker_hops_per_gap_move = 4              # count (2 blockers aside and back per knight gap move)
    adjacent_hover_crowding_factor = 2.33      # x (verification.py: full 8-neighbour hover penalty, energy-optimal commutation)
    landing_crowding_factor = 1.75             # x (fleet average over the 2x8 home blocks: 4.5 of 8 neighbours)
    control_tile_side = 100                    # mm
    piece_control_flops = 20000                # flop/update
    setpoint_dma_words_flops = 4               # flop per setpoint word (pack + DMA descriptor, FPGA does the delta-sigma)
    setpoint_fpga_price = 4.05                 # USD (GOWIN GW1NZ-LV1QN48C6 $3.90 at 100, LCSC C5799569, + 1.2V core LDO; time-multiplexed BRAM modulator must be proven in synthesis)
    setpoint_fpga_power = 0.10                 # W per tile (static + 25MHz fabric)
    setpoint_fpga_solder_joints = 48           # count (QFN-48)
    node_mcu_throughput_mflops = 170           # Mflop/s
    tile_mcu_power = 0.4                       # W
    host_power = 8                             # W
    psu_sizing_margin = 1.1                    # ratio (line/aging only: load is already governed worst-case peak with fleet-wide look-ahead)
    psu_unit_mass_kg = 0.98                    # kg
    psu_options = {
        24: ("Mean Well UHP-500-12, provisional isolated +/-12V zones", 500.4, 83.3004, "https://www.digikey.com/en/products/detail/mean-well-usa-inc/UHP-500-12/8324034"),
        30: ("Mean Well UHP-500-15, series pair = +/-15V split rail", 501.0, 94.20, "https://www.digikey.com/en/products/detail/mean-well-usa-inc/UHP-500-15/8324035"),
        48: ("Mean Well UHP-500-24, series pair = +/-24V split rail", 501.6, 94.20, "https://www.digikey.com/en/products/detail/mean-well-usa-inc/UHP-500-24/8324036"),
    }


class Constants:
    board_squares_per_side = 8
    ndfeb_remanence_br = 1.40                  # N48SH: Hcj headroom for ~70C surface soak in a Halbach; N52 knee too close
    ndfeb_density = 0.0075
    plastic_density = 0.0012
    copper_resistivity = 1.724e-08
    aluminium_resistivity = 2.82e-08
    copper_density = 8960
    winding_conductor_thicknesses = [0.05, 0.07, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5]
    standard_bus_voltages = [24, 30, 48]
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
        self.period_length = Inputs.magnets_per_period * Inputs.magnet_lateral_edge
        self.platform_side = Inputs.periods_per_side * self.period_length
        self.base_diameter = self.platform_side * sqrt(2) + 2 * Fixed.base_corner_standoff
        self.min_square_size = self.base_diameter / Fixed.square_fill_ratio
        self.periods_per_square = ceil(self.min_square_size / self.period_length)
        self.square_size = self.periods_per_square * self.period_length
        self.square_fill = self.base_diameter / self.square_size
        self.board_side = Constants.board_squares_per_side * self.square_size
        self.captured_per_side = ceil(Fixed.captured_pieces_total / Fixed.captured_side_areas)
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
        self.outer_width = board.period_length / Inputs.coils_per_period
        self.outer_length = Inputs.coil_outer_length
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
        self.conductor_radial_width = self.winding_radial_width / Fixed.turns_per_radial_layer
        self.body_density = Fixed.herringbone_orientation_families / (self.outer_width * self.outer_length)
        self.coil_spacing = sqrt(1 / self.body_density)
        self.total_bodies = ceil(board.motor_area * self.body_density)
        self.windings = self.total_bodies * Fixed.windings_per_coil_body
        self.active_bodies = self.control_bed_bodies * Inputs.pieces_levitating_simultaneously
        self.active_windings = self.active_bodies * Fixed.windings_per_coil_body
        self.peak_driven_windings = ceil(self.active_windings * Inputs.drive_look_ahead_factor)
        self.half_bridges_per_coil = 1
        self.drive_voltage_fraction = 0.5

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
        self.block_volume = Inputs.magnet_lateral_edge ** 2 * Inputs.magnet_thickness
        self.blocks_per_platform = self.blocks_per_side ** 2
        self.block_mass = self.block_volume * Constants.ndfeb_density
        self.magnet_mass = self.blocks_per_platform * self.block_mass
        self.circumradius = board.platform_side / 2 * sqrt(2)
        self.resting_magnet_gap = board.base_diameter - 2 * self.circumradius
        self.b_at_coils = sim["peak_bz"]

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
        self.worst_case_poses = sim["worst_case_poses"]
        self.worst_case_max_gap = sim["worst_case_max_gap"] * 1000
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
            Cell("Worst-case maximum flight gap", self.worst_case_max_gap, "mm"),
            Cell("Worst-case max tilt swept", self.worst_case_max_tilt_deg, "deg"),
            Cell("Worst-case lift force per amp (sim)", self.worst_force_per_amp, "N/A"),
            Cell("Worst-case required lift current", self.worst_required_current, "A"),
            Cell("Worst-case available lift margin", self.worst_available_margin, "x"),
            Cell("Worst-case hover power (one piece)", self.worst_piece_hover_power, "W"),
            Cell("Showpiece-tilt lift margin (deepest tilt)", self.showpiece_available_margin, "x"),
            Cell("Showpiece-tilt hover power (one piece)", self.showpiece_hover_power, "W"),
            Cell("Best-phase hover power (one piece)", self.best_phase_hover_power, "W"),
            Cell("Phase-averaged hover power (one piece)", self.piece_hover_power, "W"),
            Cell("Operating current per winding", self.operating_current, "A"),
            Cell("Lift force per coil body", self.force_per_body, "N"),
            Cell("Total lift force", self.total_force, "N"),
            Cell("Lift margin (with safety)", self.margin, "x"),
        ]


class ConfigurationSweep:
    def electrical_power_proxy(self, board, coil, c):
        return WireThermal(coil, c).all_pieces_power + DiscreteDriver(board, coil, c).total_power

    def first_failed_gate(self, board, coil, halbach, piece, sim, c):
        if c.inner_window_width <= 0 or c.inner_window_length <= 0:
            return "window"
        if c.available_margin < Inputs.force_safety_factor:
            return "lift"
        if c.bus_voltage > Fixed.mosfet_voltage_rating:
            return "mosfet_voltage"
        driver = DiscreteDriver(board, coil, c)
        quiet_cooling = RadiatorCooling(board, c, driver, 0, False)
        cooling = RadiatorCooling(board, c, driver, Inputs.active_cooling_fans, True)
        for mode_cooling in (quiet_cooling, cooling):
            if mode_cooling.cyclic_peak_baseplate_temp > Inputs.max_surface_temperature:
                return "baseplate_temp"
            if mode_cooling.cyclic_peak_source_temp > Inputs.coil_bed_temp_limit:
                return "source_temp"
            if mode_cooling.worst_piece_local_temp > Inputs.coil_bed_temp_limit:
                return "local_hotspot"
            if mode_cooling.worst_piece_local_temp > Fixed.magnet_max_operating_temperature:
                return "magnet_temp"
            if mode_cooling.worst_piece_local_temp > Fixed.max_touch_temperature:
                return "touch_temp"
        prop = Propulsion(board, coil, piece, c, halbach, sim)
        if prop.acceleration_in_g < Inputs.min_maneuver_accel_g:
            return "maneuvering"
        att = AttitudeAuthority(board, piece, c, sim)
        if att.tilt_margin < 1 or att.yaw_margin < 1:
            return "attitude"
        if c.worst_available_margin < Inputs.force_safety_factor:
            return "worst_lift"
        if prop.worst_acceleration_in_g < Inputs.min_maneuver_accel_g:
            return "worst_maneuvering"
        if att.worst_tilt_margin < 1 or att.worst_yaw_margin < 1:
            return "worst_attitude"
        psu = PowerSupply(coil, WireThermal(coil, c), TileControl(board, coil, Control(coil, c, sim)), c, driver, cooling)
        if psu.required_rating > psu.supply_rating:
            return "psu_rating"
        if psu.required_current > Fixed.max_bus_current:
            return "bus_current"
        return None

    def is_feasible(self, board, coil, halbach, piece, sim, c):
        return self.first_failed_gate(board, coil, halbach, piece, sim, c) is None

    def __init__(self, board, coil, halbach, piece, sim):
        self.configurations = [
            CoilConfiguration(coil, halbach, piece, sim, conductor_thickness, bus_voltage, layers)
            for bus_voltage in Constants.standard_bus_voltages
            for conductor_thickness in Constants.winding_conductor_thicknesses
            for layers in range(1, floor(coil.outer_width / (conductor_thickness + 2 * Fixed.rectangular_wire_film)) + 1)
        ]
        self.feasible = [c for c in self.configurations if self.is_feasible(board, coil, halbach, piece, sim, c)]
        if not self.feasible:
            gates = {}
            for c in self.configurations:
                gate = self.first_failed_gate(board, coil, halbach, piece, sim, c)
                gates[gate] = gates.get(gate, 0) + 1
            raise RuntimeError(f"NO FEASIBLE CONFIGURATION FOUND - first failed gate tally: {gates}")
        self.selected = min(self.feasible, key=lambda c: self.electrical_power_proxy(board, coil, c))
        self.best_per_voltage = []
        for bus_voltage in Constants.standard_bus_voltages:
            candidates = [c for c in self.feasible if c.bus_voltage == bus_voltage]
            if candidates:
                self.best_per_voltage.append(min(candidates, key=lambda c: self.electrical_power_proxy(board, coil, c)))


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
        self.control_bits = self.channels
        self.gate_drivers = ceil(self.half_bridges / Fixed.gate_driver_half_bridges)
        self.tile_count = ceil(board.motor_width / Fixed.control_tile_side) * ceil(board.motor_height / Fixed.control_tile_side)
        self.channels_per_tile = ceil(self.channels / self.tile_count)
        self.control_bits_per_tile = self.channels_per_tile
        self.shift_registers_per_tile = ceil(self.control_bits_per_tile / Fixed.shift_register_outputs)
        self.shift_registers = self.shift_registers_per_tile * self.tile_count
        self.serial_clock = min(Fixed.driver_serial_clock, Fixed.shift_register_clock_rating)
        self.serial_data_rate = self.serial_clock
        self.serial_headroom = self.serial_clock / self.serial_data_rate
        self.setpoint_frame_rate = self.serial_clock / self.control_bits_per_tile
        self.setpoint_oversampling_ratio = self.setpoint_frame_rate / (2 * Fixed.setpoint_filter_cutoff)
        self.effective_setpoint_bits = min(Fixed.max_setpoint_bits, (15 * log2(self.setpoint_oversampling_ratio) - 13) / 6.02)
        self.setpoint_resolution = Fixed.driver_channel_current / 2 ** self.effective_setpoint_bits
        self.current_squared_sum = config.piece_hover_power * Inputs.pieces_levitating_simultaneously / config.resistance
        self.current_sum_upper_bound = sqrt(self.active_channels * self.current_squared_sum)
        self.current_offset_error = Fixed.current_sense_offset_residual / Fixed.current_sense_resistance
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
            Cell("Driver implementation", "current-regulated N-MOSFET half-bridges, split rail"),
            Cell("Driver control implementation", "per-tile FPGA delta-sigma setpoint modulators + midpoint-referenced comparator current loop, idle zero-calibrated"),
            Cell("Dedicated driver channels", self.channels),
            Cell("Current feedback channels", self.current_feedback_channels),
            Cell("Discrete half-bridge legs", self.half_bridges),
            Cell("Gate-driver ICs", self.gate_drivers),
            Cell("Driver current limit", Fixed.driver_channel_current, "A"),
            Cell("MOSFET voltage rating", Fixed.mosfet_voltage_rating, "V"),
            Cell("Comparator current-offset error", self.current_offset_error * 1000, "mA"),
            Cell("MOSFET gate drive", Fixed.gate_drive_voltage, "V"),
            Cell("PWM frequency", Fixed.driver_pwm_frequency / 1000, "kHz"),
            Cell("Setpoint frame rate (delta-sigma)", self.setpoint_frame_rate / 1000, "kHz"),
            Cell("Setpoint oversampling ratio", self.setpoint_oversampling_ratio, "x"),
            Cell("Effective setpoint bits (delta-sigma)", self.effective_setpoint_bits),
            Cell("Current command refresh", Fixed.current_command_rate, "Hz"),
            Cell("Current loop bandwidth limit", Fixed.current_loop_bandwidth, "Hz"),
            Cell("Comparator offset (raw / after zero-cal)", f"{Fixed.comparator_input_offset * 1000:g} / {Fixed.current_sense_offset_residual * 1000:g} mV"),
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
    def piecewise_peak_rise(self, segments, resistance, time_constant):
        temperature = 0.0
        peak = 0.0
        for duration, power in segments:
            settled = power * resistance
            temperature = settled + (temperature - settled) * exp(-duration / time_constant)
            peak = max(peak, temperature)
        return peak

    def __init__(self, board, config, driver, fan_count, grind_baseline):
        self.fan_count = fan_count
        self.grind_baseline = grind_baseline
        self.mode = ("fan-assisted" if fan_count else "passive") + ", " + ("real-time replay baseline" if grind_baseline else "live-play baseline")
        self.board_area = board.motor_area / 1000000
        self.piece_footprint_area = pi / 4 * (board.base_diameter / 1000) ** 2
        self.source_area = min(self.board_area, self.piece_footprint_area * Inputs.pieces_levitating_simultaneously)
        self.coil_bed_thickness = Fixed.herringbone_orientation_families * config.coil_height
        self.stack_area_resistance = (
            self.coil_bed_thickness / 1000 / 2 / Fixed.coil_bed_through_conductivity
            + Fixed.potting_thickness / 1000 / Fixed.potting_thermal_conductivity
            + Fixed.pcb_thickness / 1000 / Fixed.pcb_via_effective_thermal_conductivity
            + Fixed.radiator_standoff_below_pcb / 1000 / Fixed.thermal_pad_conductivity
            + Fixed.baseplate_thickness / 1000 / Fixed.aluminium_thermal_conductivity
        )
        self.source_to_baseplate_resistance = self.stack_area_resistance / self.source_area
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
        self.driver_loss_ratio = self.driver_power / self.coil_power
        fleet = Inputs.pieces_levitating_simultaneously
        loss_scale = 1 + self.driver_loss_ratio
        self.event_segments = [
            (Inputs.worst_phase_dwell, fleet * config.worst_piece_hover_power * loss_scale),
            (Inputs.cruise_duration, fleet * config.piece_hover_power * Inputs.drive_look_ahead_factor * loss_scale),
            (Inputs.contention_hover, fleet * config.best_phase_hover_power * loss_scale),
            (Inputs.landing_dwell, fleet * config.best_phase_hover_power * Fixed.landing_crowding_factor * loss_scale),
        ]
        self.event_duration = sum(duration for duration, _ in self.event_segments)
        self.event_energy = sum(duration * power for duration, power in self.event_segments)
        self.event_average_power = self.event_energy / self.event_duration
        self.event_peak_power = max(power for _, power in self.event_segments)
        self.play_move_energy = (
            (Inputs.worst_phase_dwell + Inputs.landing_dwell) * config.best_phase_hover_power
            + Inputs.cruise_duration * config.piece_hover_power * Inputs.drive_look_ahead_factor
        ) * loss_scale
        self.play_average_power = self.play_move_energy / Inputs.play_move_period
        self.blocker_hop_energy = (Inputs.worst_phase_dwell + Inputs.landing_dwell) * config.best_phase_hover_power * loss_scale
        self.composite_move_energy = (
            self.play_move_energy * (1 + Inputs.replay_capture_fraction)
            + Inputs.replay_knight_gap_fraction * Fixed.blocker_hops_per_gap_move * self.blocker_hop_energy
        )
        self.grind_power = self.composite_move_energy * Inputs.sustained_moves_per_minute / 60
        self.sustained_power = self.grind_power if grind_baseline else self.play_average_power
        self.baseline_rise = self.sustained_power / self.thermal_conductance
        self.baseplate_rise_per_watt = (
            1 / self.thermal_conductance
            * (1 - exp(-self.event_duration / self.thermal_time_constant))
            / (1 - exp(-Inputs.sustained_event_period / self.thermal_time_constant))
        )
        self.event_cyclic_rise = self.event_average_power * self.baseplate_rise_per_watt
        self.back_to_back_rise = (Inputs.events_back_to_back - 1) * self.event_energy / self.thermal_capacitance
        self.cyclic_peak_baseplate_rise = self.baseline_rise + self.event_cyclic_rise + self.back_to_back_rise
        self.cyclic_peak_baseplate_temp = Inputs.ambient_temperature + self.cyclic_peak_baseplate_rise
        self.stack_area_capacitance = self.coil_bed_thickness / 1000 * Fixed.potting_volumetric_heat_capacity
        self.source_time_constant = self.stack_area_resistance * self.stack_area_capacitance
        self.source_event_rise = self.piecewise_peak_rise(self.event_segments * Inputs.events_back_to_back,
                                                          self.source_to_baseplate_resistance, self.source_time_constant)
        self.cyclic_peak_source_temp = self.cyclic_peak_baseplate_temp + self.source_event_rise
        self.local_resistance = self.stack_area_resistance / self.piece_footprint_area
        self.local_heat_capacity = self.stack_area_capacitance * self.piece_footprint_area
        self.hammer_local_rise = config.best_phase_hover_power * Inputs.hammer_dwell / Inputs.hammer_visit_period * self.local_resistance
        self.exchange_hover_time = min(2 * Inputs.hammer_dwell, 60 / Inputs.sustained_moves_per_minute)
        self.cascade_local_rise = self.piecewise_peak_rise(
            [(Inputs.cascade_exchanges * self.exchange_hover_time, config.best_phase_hover_power)],
            self.local_resistance, self.source_time_constant)
        self.takeoff_local_rise = self.piecewise_peak_rise([(Inputs.worst_phase_dwell, config.worst_piece_hover_power)],
                                                           self.local_resistance, self.source_time_constant)
        self.worst_piece_local_rise = self.hammer_local_rise + self.cascade_local_rise + self.takeoff_local_rise
        self.worst_piece_local_temp = self.cyclic_peak_baseplate_temp + self.worst_piece_local_rise
        self.baseplate_margin = (Inputs.max_surface_temperature - Inputs.ambient_temperature) / self.cyclic_peak_baseplate_rise
        self.source_margin = (Inputs.coil_bed_temp_limit - Inputs.ambient_temperature) / (self.cyclic_peak_source_temp - Inputs.ambient_temperature)
        self.local_margin = (Inputs.coil_bed_temp_limit - Inputs.ambient_temperature) / (self.worst_piece_local_temp - Inputs.ambient_temperature)
        self.thermal_margin = min(self.baseplate_margin, self.source_margin, self.local_margin)
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
            Cell("Coil heat (32 pieces, phase-averaged)", self.coil_power, "W"),
            Cell("Driver/control heat (phase-averaged)", self.driver_power, "W"),
            Cell("Driver loss ratio (per coil watt)", self.driver_loss_ratio, "x"),
            Cell("Reset event duration", self.event_duration, "s"),
            Cell("Reset event heat energy", self.event_energy, "J"),
            Cell("Reset event peak heat (worst-phase lift)", self.event_peak_power, "W"),
            Cell("Reset event average heat", self.event_average_power, "W"),
            Cell("Live-play move energy (one piece)", self.play_move_energy, "J"),
            Cell("Blocker hop energy (gap-making)", self.blocker_hop_energy, "J"),
            Cell("Composite replay move energy (mix)", self.composite_move_energy, "J"),
            Cell("W2 sustained baseline heat (this mode)", self.sustained_power, "W"),
            Cell("Source area (32 piece footprints)", self.source_area * 10000, "cm2"),
            Cell("Coil-bed conduction thickness", self.coil_bed_thickness, "mm"),
            Cell("Source-to-baseplate resistance", self.source_to_baseplate_resistance, "K/W"),
            Cell("Source RC time constant", self.source_time_constant, "s"),
            Cell("Bottom fins", self.fin_count),
            Cell("Fin height", self.fin_height * 1000, "mm"),
            Cell("Fin channel width", Fixed.fin_channel_width, "mm"),
            Cell("Effective convection area", self.convection_area, "m2"),
            Cell("Convection coefficient", self.convection_coefficient, "W/(m2.K)"),
            Cell("Radiator aluminium mass", self.aluminium_mass, "kg"),
            Cell("Thermal time constant", self.thermal_time_constant, "s"),
            Cell("W2 baseline rise (sustained duty)", self.baseline_rise, "K"),
            Cell("Sustained-reset cyclic rise", self.event_cyclic_rise, "K"),
            Cell("Back-to-back reset extra rise", self.back_to_back_rise, "K"),
            Cell("Cyclic peak baseplate temp", self.cyclic_peak_baseplate_temp, "C"),
            Cell("Source rise (fleet, double reset)", self.source_event_rise, "K"),
            Cell("Cyclic peak coil/MOSFET temp", self.cyclic_peak_source_temp, "C"),
            Cell("W3 hammer local rise (dwell / 5s forever)", self.hammer_local_rise, "K"),
            Cell("Cascade hover per exchange (serialized)", self.exchange_hover_time, "s"),
            Cell("W3 cascade local rise (RC-drained burst)", self.cascade_local_rise, "K"),
            Cell("Worst-phase takeoff local rise", self.takeoff_local_rise, "K"),
            Cell("Worst-case local coil temp (one cell)", self.worst_piece_local_temp, "C"),
            Cell("Thermal margin (plate/source/local min)", self.thermal_margin, "x"),
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
            (thermal.worst_piece_local_temp - Inputs.ambient_temperature)
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
            Cell("Touch policy", "governor hard-caps every cell at the brief-touch limit; any square a hand can reach is safe the instant it is uncovered"),
        ]


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
        surface_stack = Fixed.potting_cover_thickness + Fixed.playing_surface_thickness + Fixed.piece_bottom_skin
        self.tilt_angle = asin((Inputs.max_flight_gap - surface_stack - Inputs.tilt_rim_clearance) / (piece.diameter / 2))
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

    def required_tilt_torque(self, angle):
        return (self.mass * Constants.gravity * self.com_height * sin(angle)
                + self.tilt_inertia * 4 * angle / Inputs.target_tilt_time ** 2)

    def cells(self):
        return [
            Cell("Tilt lever arm (footprint)", self.lever_arm * 1000, "mm"),
            Cell("Max tilt torque available (sim)", self.tilt_torque_max, "N.m"),
            Cell("Geometric tilt cap (1mm rim clearance at showpiece gap)", degrees(self.tilt_angle), "deg"),
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


class CoupledAuthority:
    def __init__(self, coil, piece, config, attitude):
        self.ampere_turn_budget = config.turns * config.current_limit
        authority = levitation_sim.verified_worst_authority(coil.control_cells_per_side, config.coil_height / 1000, self.ampere_turn_budget)
        self.lift_margin = authority["lift_margin"]
        self.showpiece_lift_margin = authority["showpiece_lift_margin"]
        self.lateral_force = authority["lateral"]
        self.level_lateral_force = authority["level_lateral"]
        self.acceleration_in_g = self.lateral_force / (piece.mass / 1000) / Constants.gravity
        self.level_acceleration_in_g = self.level_lateral_force / (piece.mass / 1000) / Constants.gravity
        self.tilt_torque = authority["tilt"]
        self.yaw_torque = authority["yaw"]
        self.yaw_margin = self.yaw_torque / attitude.yaw_required_torque
        self.rungs = []
        for fraction in sorted(authority["rungs"], reverse=True):
            rung = authority["rungs"][fraction]
            required = attitude.required_tilt_torque(radians(rung["tilt_deg"]))
            self.rungs.append((rung["tilt_deg"], rung["lift_margin"], rung["tilt_torque"] / required))
        self.design_tilt_deg, self.design_lift_margin, self.design_tilt_margin = self.rungs[-1]
        for tilt_deg, lift_margin, tilt_margin in self.rungs:
            if lift_margin >= Inputs.force_safety_factor and tilt_margin >= 1:
                self.design_tilt_deg, self.design_lift_margin, self.design_tilt_margin = tilt_deg, lift_margin, tilt_margin
                break
        self.tilt_margin = self.design_tilt_margin

    def cells(self):
        rows = [
            Cell("Ampere-turn budget per coil (driver limit)", self.ampere_turn_budget, "A.t"),
            Cell("Verified worst-case lift margin (level transport)", self.lift_margin, "x"),
            Cell("Coupled level-pose lateral force (transport)", self.level_lateral_force, "N"),
            Cell("Coupled level-pose lateral acceleration", self.level_acceleration_in_g, "g"),
            Cell("Coupled worst yaw torque (hover held)", self.yaw_torque, "N.m"),
            Cell("Coupled worst yaw margin", self.yaw_margin, "x"),
        ]
        for tilt_deg, lift_margin, tilt_margin in self.rungs:
            rows.append(Cell(f"Showpiece rung {tilt_deg:.1f} deg: lift / tilt-torque margin",
                             f"{lift_margin:.2f} / {tilt_margin:.2f} x"))
        rows.append(Cell("Selected showpiece tilt (deepest affordable)", self.design_tilt_deg, "deg"))
        rows.append(Cell("Showpiece lift margin (selected rung)", self.design_lift_margin, "x"))
        rows.append(Cell("Showpiece tilt-torque margin (selected rung)", self.design_tilt_margin, "x"))
        return rows


class EddyDrag:
    def sheet(self, name, depth_mm, resistivity, thickness_mm, speed, loop_factor):
        characteristic_velocity = 2 * resistivity / (Constants.vacuum_permeability * thickness_mm / 1000)
        image_force = levitation_sim.eddy_image_force(-depth_mm / 1000)
        drag = abs(image_force) * speed * characteristic_velocity / (speed ** 2 + characteristic_velocity ** 2) * loop_factor
        damping = abs(image_force) / characteristic_velocity * loop_factor
        return name, depth_mm, characteristic_velocity, abs(image_force), drag, damping

    def __init__(self, board, config, piece, coupled):
        self.mass = piece.mass / 1000
        self.coupled_acceleration = coupled.level_lateral_force / self.mass
        self.traverse_distance = (Constants.board_squares_per_side - 1) * board.square_size / 1000
        self.design_speed = sqrt(self.coupled_acceleration * self.traverse_distance)
        self.slot_factor = min(1.0, (Fixed.radiator_slot_pitch / (board.period_length / 2)) ** 2)
        pcb_plane_depth = Inputs.magnet_to_coil_distance + 2 * config.coil_height + Fixed.pcb_thickness / 2
        plate_depth = (Inputs.magnet_to_coil_distance + 2 * config.coil_height + Fixed.pcb_thickness
                       + Fixed.radiator_standoff_below_pcb)
        slotted_thickness = Fixed.baseplate_thickness - Fixed.radiator_slot_web_thickness
        web_depth = plate_depth + slotted_thickness
        self.sheet_specs = [
            ("PCB copper planes", pcb_plane_depth, Constants.copper_resistivity, Fixed.pcb_copper_plane_thickness, 1.0),
            ("Radiator plate (crosshatch-slotted)", plate_depth, Constants.aluminium_resistivity, slotted_thickness, self.slot_factor),
            ("Radiator solid bottom web", web_depth, Constants.aluminium_resistivity, Fixed.radiator_slot_web_thickness, 1.0),
        ]
        self.sheets = [self.sheet(name, depth, rho, thick, self.design_speed, loop)
                       for name, depth, rho, thick, loop in self.sheet_specs]
        self.total_drag = sum(s[4] for s in self.sheets)
        self.total_damping = sum(s[5] for s in self.sheets)
        self.drag_to_thrust = self.total_drag / coupled.level_lateral_force
        self.eddy_heating = self.total_drag * self.design_speed
        self.damping_rate = self.total_damping / self.mass
        self.cruise_distance = sqrt(board.motor_width ** 2 + board.motor_height ** 2) / 1000
        self.cruise_accel_required = 4 * self.cruise_distance / Inputs.cruise_duration ** 2
        self.cruise_peak_speed = 2 * self.cruise_distance / Inputs.cruise_duration
        self.slide_distance = board.square_size / 2 / 1000
        self.slide_accel_required = 4 * self.slide_distance / Inputs.worst_phase_dwell ** 2
        self.slide_peak_speed = 2 * self.slide_distance / Inputs.worst_phase_dwell
        self.cruise_drag = self.drag_at(self.cruise_peak_speed)
        self.slide_drag = self.drag_at(self.slide_peak_speed)
        self.cruise_thrust_required = self.bang_bang_thrust(self.cruise_distance, Inputs.cruise_duration)
        self.slide_thrust_required = self.bang_bang_thrust(self.slide_distance, Inputs.worst_phase_dwell)
        self.cruise_margin = coupled.level_lateral_force / self.cruise_thrust_required
        self.slide_margin = coupled.level_lateral_force / self.slide_thrust_required

    def bang_bang_thrust(self, distance, window):
        k = self.total_damping / self.mass
        def arrival_mismatch(switch):
            return switch + log(2 - exp(-k * switch)) / k - window
        lo, hi = 0.0, window
        for _ in range(80):
            mid = (lo + hi) / 2
            if arrival_mismatch(mid) < 0:
                lo = mid
            else:
                hi = mid
        switch = (lo + hi) / 2
        terminal_speed = 1 / self.total_damping
        decay = exp(-k * switch)
        switch_speed = terminal_speed * (1 - decay)
        accel_distance = terminal_speed * (switch - (1 - decay) / k)
        brake_time = window - switch
        brake_decay = exp(-k * brake_time)
        brake_distance = -terminal_speed * brake_time + (switch_speed + terminal_speed) * (1 - brake_decay) / k
        return distance / (accel_distance + brake_distance)

    def drag_at(self, speed):
        return sum(self.sheet(name, depth, rho, thick, speed, loop)[4]
                   for name, depth, rho, thick, loop in self.sheet_specs)

    def cells(self):
        rows = [
            Cell("Design traverse speed (coupled accel)", self.design_speed, "m/s"),
            Cell("Crosshatch slot pitch (island size)", Fixed.radiator_slot_pitch, "mm"),
            Cell("Eddy loop-area slot factor", self.slot_factor, "x"),
        ]
        for name, depth, velocity, image_force, drag, damping in self.sheets:
            rows.append(Cell(f"{name}: depth below magnets", depth, "mm"))
            rows.append(Cell(f"{name}: characteristic velocity", velocity, "m/s"))
            rows.append(Cell(f"{name}: image (superconductor) force", image_force * 1000, "mN"))
            rows.append(Cell(f"{name}: drag at design speed", drag * 1000, "mN"))
        rows.extend([
            Cell("Total eddy drag at design speed", self.total_drag * 1000, "mN"),
            Cell("Drag fraction of coupled lateral thrust", self.drag_to_thrust, "x"),
            Cell("Total eddy damping coefficient", self.total_damping, "N.s/m"),
            Cell("Eddy damping rate (c/m)", self.damping_rate, "1/s"),
            Cell("Eddy heating at design speed", self.eddy_heating * 1000, "mW"),
            Cell("Cruise scenario (level, nominal gap, full diagonal)", f"{self.cruise_distance * 1000:.0f} mm in {Inputs.cruise_duration:g} s"),
            Cell("Cruise accel required", self.cruise_accel_required / Constants.gravity, "g"),
            Cell("Cruise thrust required (bang-bang incl. drag)", self.cruise_thrust_required * 1000, "mN"),
            Cell("Cruise authority margin", self.cruise_margin, "x"),
            Cell("Slide-to-phase scenario (worst pose)", f"{self.slide_distance * 1000:.0f} mm in {Inputs.worst_phase_dwell:g} s"),
            Cell("Slide accel required", self.slide_accel_required / Constants.gravity, "g"),
            Cell("Slide thrust required (bang-bang incl. drag)", self.slide_thrust_required * 1000, "mN"),
            Cell("Slide authority margin", self.slide_margin, "x"),
        ])
        return rows


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
        self.scheme = "half-bridge per coil, center-tapped split rail (bipolar, +/-Vbus/2 swing)"
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
        self.current_command_resolution = driver.setpoint_resolution
        self.current_offset_error = driver.current_offset_error
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
    def __init__(self, coil, config, control, tiles):
        self.update_rate = control.pose_update_rate
        self.sensor_pitch = tiles.hall_pitch
        self.sensors_per_piece = Fixed.hall_observation_window_side ** 2
        self.sensors_per_tile = tiles.hall_sensors_per_tile
        self.total_sensors = tiles.hall_total_sensors
        self.muxes_per_tile = tiles.hall_muxes_per_tile
        self.oversampling_factor = tiles.hall_oversampling_factor
        self.reads_per_tile = self.sensors_per_tile * self.update_rate * self.oversampling_factor
        self.tile_capacity = Fixed.hall_adc_sample_rate
        self.headroom = 1 / tiles.hall_scan_fraction
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
        self.interference_bias_um = self.position_noise_gain * (self.coil_field_hover + self.neighbour_field) * Fixed.coil_field_subtraction_error * 1e6
        self.total_position_error_um = self.position_noise_um + self.interference_bias_um
        self.required_position_error_um = Inputs.magnet_to_coil_distance * 1000 * Fixed.position_error_gap_fraction
        self.saturation_field = self.signal_peak + self.coil_field_hover_bound

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
            Cell("Hall linear range", Fixed.hall_linear_range * 1000, "mT"),
            Cell("ADC quantization (field)", self.adc_quantization_field * 1e6, "uT"),
            Cell("Effective field noise per update", self.field_noise * 1e6, "uT"),
            Cell("Worst position noise gain", self.position_noise_gain, "m/T"),
            Cell("Position noise (worst pose)", self.position_noise_um, "um"),
            Cell("Tilt noise (worst pose)", self.tilt_noise_mrad, "mrad"),
            Cell("Interference bias after subtraction", self.interference_bias_um, "um"),
            Cell("Total position error", self.total_position_error_um, "um"),
            Cell("Position error budget (gap fraction)", self.required_position_error_um, "um"),
            Cell("Sensors per tile", self.sensors_per_tile),
            Cell("Total Hall sensors (board)", self.total_sensors),
            Cell("Hall array supply power", self.total_sensors * Fixed.hall_supply_current * Fixed.hall_supply_voltage, "W"),
            Cell("Readout muxes per tile", self.muxes_per_tile),
            Cell("ADC oversampling factor", self.oversampling_factor, "x"),
            Cell("Independent sensor-noise averages", self.sensor_noise_averages, "x"),
            Cell("Sensor averaging group delay", self.averaging_group_delay_ms, "ms"),
            Cell("Reads needed per tile", self.reads_per_tile, "reads/s"),
            Cell("Per-tile ADC capacity", self.tile_capacity, "samples/s"),
            Cell("Scan headroom (incl. gating settle)", self.headroom, "x"),
        ]


class TileControl:
    def __init__(self, board, coil, control, hall_pitch=Fixed.hall_sensor_pitch):
        self.hall_pitch = hall_pitch
        self.tile_side = Fixed.control_tile_side
        self.tiles_per_width = ceil(board.motor_width / self.tile_side)
        self.tiles_per_height = ceil(board.motor_height / self.tile_side)
        self.tile_count = self.tiles_per_width * self.tiles_per_height
        self.coils_per_tile = ceil(coil.total_bodies / self.tile_count)
        self.hall_sensors_per_tile_side = ceil(Fixed.control_tile_side / hall_pitch)
        self.hall_sensors_per_tile = self.hall_sensors_per_tile_side ** 2
        self.hall_total_sensors = self.hall_sensors_per_tile * self.tile_count
        self.hall_muxes_per_tile = ceil(self.hall_sensors_per_tile / Fixed.hall_sensor_mux_channels)
        self.hall_oversampling_factor = 4 ** max(0, Fixed.hall_interpolation_bits - Fixed.hall_adc_native_bits)
        self.hall_group_burst_time = Fixed.hall_sensor_mux_channels * self.hall_oversampling_factor / Fixed.hall_adc_sample_rate
        self.hall_gating_duty = (Fixed.hall_power_settle_time + self.hall_group_burst_time) * control.pose_update_rate
        self.hall_scan_fraction = self.hall_muxes_per_tile * self.hall_gating_duty
        self.hall_peak_supply_power = self.hall_total_sensors * Fixed.hall_supply_current * Fixed.hall_supply_voltage
        self.hall_supply_power = self.hall_peak_supply_power * self.hall_gating_duty
        self.square_area = board.square_size ** 2
        self.max_pieces_per_tile = max(1, ceil(self.tile_side ** 2 / self.square_area))
        self.pose_rate = control.pose_update_rate
        self.node_capacity = Fixed.node_mcu_throughput_mflops * 1e6
        self.setpoint_stream_compute = self.coils_per_tile * Fixed.current_command_rate * Fixed.setpoint_dma_words_flops
        self.tile_compute = self.max_pieces_per_tile * Fixed.piece_control_flops * self.pose_rate + self.setpoint_stream_compute
        self.central_compute = Inputs.pieces_levitating_simultaneously * Fixed.piece_control_flops * self.pose_rate
        self.tile_headroom = self.node_capacity / self.tile_compute
        self.central_headroom = self.node_capacity / self.central_compute

    def cells(self):
        return [
            Cell("Control tile side (square)", self.tile_side, "mm"),
            Cell("Hall pitch (cost-selected, sparsest passing)", self.hall_pitch, "mm"),
            Cell("Control tiles (count)", self.tile_count),
            Cell("Coils per tile", self.coils_per_tile),
            Cell("Max pieces over one tile", self.max_pieces_per_tile),
            Cell("Setpoint architecture", "per-tile FPGA delta-sigma modulators; MCU DMA-streams 12-bit setpoints"),
            Cell("Setpoint-stream MCU load (DMA)", self.setpoint_stream_compute / 1e6, "Mflop/s"),
            Cell("Per-tile compute load", self.tile_compute / 1e6, "Mflop/s"),
            Cell("Per-tile MCU capacity", self.node_capacity / 1e6, "Mflop/s"),
            Cell("Per-tile compute headroom", self.tile_headroom, "x"),
            Cell("Single-MCU compute headroom (rejected)", self.central_headroom, "x"),
            Cell("Hall gating policy", "per-mux-group high-side switch, powered only during scan burst"),
            Cell("Hall group burst time", self.hall_group_burst_time * 1000, "ms"),
            Cell("Hall gating duty", self.hall_gating_duty, "x"),
            Cell("Hall supply power (peak, all on)", self.hall_peak_supply_power, "W"),
            Cell("Hall supply power (gated)", self.hall_supply_power, "W"),
        ]


class EnergyBuffer:
    def __init__(self, bus_voltage, zones, deficit_power, deficit_energy):
        self.rail_voltage = bus_voltage / 2
        self.series_cells = ceil(self.rail_voltage / Fixed.supercap_cell_working_voltage)
        self.string_capacitance = Fixed.supercap_cell_capacitance / self.series_cells
        self.string_resistance = Fixed.supercap_cell_esr * self.series_cells
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
        self.oring_count = 2 * zones
        self.price = (self.cell_count * (Fixed.supercap_cell_price + Fixed.supercap_balancer_price_per_cell)
                      + self.oring_count * Fixed.supercap_oring_price
                      + 2 * Fixed.supercap_management_price_per_rail)
        self.mass = self.cell_count * Fixed.supercap_cell_mass_kg

    def cells(self):
        return [
            Cell("Buffer topology", "one supercap bank per rail polarity, ideal-diode ORed into every zone rail (common midpoint ground)"),
            Cell("Supercap cell", f"{Fixed.supercap_cell_capacitance:g}F {Fixed.supercap_cell_max_voltage:g}V, run at {Fixed.supercap_cell_working_voltage:g}V"),
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
    def __init__(self, coil, wire, tiles, config, driver, thermal):
        self.bus_voltage = config.bus_voltage
        self.coil_lift_power = wire.all_pieces_power
        self.thrust_factor = coil.peak_driven_windings / coil.active_windings
        self.coil_peak_power = self.coil_lift_power * self.thrust_factor
        self.driver_peak_power = driver.total_power * self.thrust_factor
        self.electronics_power = (tiles.tile_count * (Fixed.tile_mcu_power + Fixed.setpoint_fpga_power)
                                  + Fixed.host_power
                                  + tiles.hall_supply_power
                                  + thermal.fan_power)
        self.total_load = self.coil_peak_power + self.driver_peak_power + self.electronics_power
        self.sustained_load = thermal.sustained_power + self.electronics_power
        self.required_rating = self.sustained_load * Fixed.psu_sizing_margin
        self.psu_family, self.unit_rating, self.unit_price, self.psu_url = Fixed.psu_options[config.bus_voltage]
        self.event_peak_load = thermal.event_peak_power + self.electronics_power
        self.event_energy = thermal.event_energy + self.electronics_power * thermal.event_duration
        self.recharge_window = Inputs.sustained_event_period - Inputs.events_back_to_back * thermal.event_duration
        fleet = Inputs.pieces_levitating_simultaneously
        peak_units = 2 * ceil(self.total_load * Fixed.psu_sizing_margin / (2 * self.unit_rating))
        best = None
        for unit_count in range(2, peak_units + 2, 2):
            supply = unit_count * self.unit_rating
            if supply < self.required_rating:
                continue
            zones = unit_count // 2
            zone_capacity = (supply / zones - (self.electronics_power + self.driver_peak_power) / zones) / config.best_phase_hover_power
            if zone_capacity < fleet:
                continue
            buffer_energy = Inputs.events_back_to_back * sum(
                max(0.0, power + self.electronics_power - supply) * duration
                for duration, power in thermal.event_segments)
            deficit_power = max(0.0, self.event_peak_load - supply)
            buffer = EnergyBuffer(self.bus_voltage, zones, deficit_power, buffer_energy)
            if buffer_energy > (supply - self.sustained_load) * self.recharge_window:
                continue
            cost = unit_count * self.unit_price + buffer.price
            if best is None or cost < best[0]:
                best = (cost, unit_count, buffer, buffer_energy, deficit_power)
        if best is None:
            self.required_rating = float("inf")
            self.supply_rating = 0.0
            self.required_current = float("inf")
            return
        self.power_architecture_cost, self.unit_count, self.buffer, self.buffer_energy, self.burst_deficit_power = best
        self.psu_part = f"{self.unit_count}x {self.psu_family}"
        self.supply_rating = self.unit_count * self.unit_rating
        self.psu_price = self.unit_count * self.unit_price
        self.peak_current = self.total_load / self.bus_voltage
        self.required_current = self.required_rating / self.bus_voltage
        self.rated_current = self.supply_rating / self.bus_voltage
        self.load_fraction = self.required_rating / self.supply_rating
        self.worst_phase_piece_capacity = (self.supply_rating - self.electronics_power - self.driver_peak_power) / config.worst_piece_hover_power
        self.zones = self.unit_count // 2
        self.zone_rating = self.supply_rating / self.zones
        self.aligned_rest_hover_power = config.best_phase_hover_power
        self.zone_hover_piece_capacity = (self.zone_rating - (self.electronics_power + self.driver_peak_power) / self.zones) / self.aligned_rest_hover_power
        self.zone_required_pieces = fleet
        self.rail_imbalance_current = sqrt(coil.active_windings) * driver.current_offset_error
        self.unit_rated_current = self.unit_rating / (self.bus_voltage / 2)
        self.recharge_time = self.buffer_energy / (self.supply_rating - self.sustained_load)
        self.burst_bus_current = self.event_peak_load / self.bus_voltage

    def cells(self):
        return [
            Cell("Coil lift power (32 pieces)", self.coil_lift_power, "W"),
            Cell("Coil peak power (+thrust)", self.coil_peak_power, "W"),
            Cell("Discrete driver peak loss", self.driver_peak_power, "W"),
            Cell("Electronics overhead", self.electronics_power, "W"),
            Cell("Total peak load (cruise, info)", self.total_load, "W"),
            Cell("Sustained load (grind + electronics)", self.sustained_load, "W"),
            Cell("Required PSU rating (sustained +margin)", self.required_rating, "W"),
            Cell("PSU sizing policy", "PSU covers sustained + recharge; supercap buffer covers every event segment above rating; unit count = min(PSU + buffer cost)"),
            Cell(f"Selected PSU ({self.psu_part})", self.supply_rating, "W"),
            Cell("PSU bank price", self.psu_price, "USD"),
            Cell("PSU + buffer architecture cost", self.power_architecture_cost, "USD"),
            Cell("Peak bus current", self.peak_current, "A"),
            Cell("Required bus current (+margin)", self.required_current, "A"),
            Cell("PSU output current", self.rated_current, "A"),
            Cell("PSU load fraction", self.load_fraction, "x"),
            Cell("C2 power policy", "thermal-governor admission; simultaneous reset burst served from PSU + supercap buffer"),
            Cell("Reset event peak load (all 32, worst phase)", self.event_peak_load, "W"),
            Cell("Reset event energy (incl. electronics)", self.event_energy, "J"),
            Cell("Burst power deficit vs PSU", self.burst_deficit_power, "W"),
            Cell("Burst energy for buffer (double reset)", self.buffer_energy, "J"),
            Cell("Buffer recharge time (PSU headroom)", self.recharge_time, "s"),
            Cell("Buffer recharge window (event period)", self.recharge_window, "s"),
            Cell("Burst bus current (transient, buffer-fed)", self.burst_bus_current, "A"),
            Cell("Pieces at worst-phase hover PSU sustains", self.worst_phase_piece_capacity),
            Cell("PSU topology", f"independent isolated series pairs, {self.zones} zones, +/-{self.bus_voltage / 2:g}V split rail each; buffer shared via rail ORing"),
            Cell("Independent PSU zones", self.zones),
            Cell("Zone rating", self.zone_rating, "W"),
            Cell("Aligned-rest hover power (one piece)", self.aligned_rest_hover_power, "W"),
            Cell("Zone hover capacity (pieces, aligned rest)", self.zone_hover_piece_capacity),
            Cell("Worst-case zone pieces (reset, full pile-up)", self.zone_required_pieces),
            Cell("Zone governor policy", "per-zone thermal-governor admission; aligned-rest hover keeps zones within capacity"),
            Cell("Split-rail policy", "zero-net-current row in commutation LP; rails carry only the offset residual imbalance"),
            Cell("Rail imbalance current (offset RSS)", self.rail_imbalance_current, "A"),
            Cell("PSU unit rated current", self.unit_rated_current, "A"),
        ]


class Stability:
    def __init__(self, board, piece, control, sensing, sim):
        self.mass = piece.mass / 1000
        self.height = piece.box_height / 1000
        self.half_width = board.platform_side / 2 / 1000
        self.vertical_stiffness = sim["vertical_stiffness"]
        self.bounce_frequency = sqrt(self.vertical_stiffness / self.mass) / (2 * pi)
        self.tilt_stiffness = self.vertical_stiffness * self.half_width ** 2
        self.tilt_inertia = self.mass * self.height ** 2 / 3
        self.rock_frequency = sqrt(self.tilt_stiffness / self.tilt_inertia) / (2 * pi)
        self.control_margin_over_rock = control.required_bandwidth / self.rock_frequency
        self.tilt_sense_resolution = sensing.tilt_noise_mrad / 1000
        self.tip_sense_resolution = self.tilt_sense_resolution * self.height

    def cells(self):
        return [
            Cell("Vertical magnetic stiffness", self.vertical_stiffness, "N/m"),
            Cell("Vertical bounce frequency", self.bounce_frequency, "Hz"),
            Cell("Tilt (rock) stiffness", self.tilt_stiffness, "N.m/rad"),
            Cell("Tip moment of inertia", self.tilt_inertia, "kg.m2"),
            Cell("King rocking frequency", self.rock_frequency, "Hz"),
            Cell("Control bandwidth over rock mode", self.control_margin_over_rock, "x"),
            Cell("Tilt sense resolution (measured)", self.tilt_sense_resolution * 1000, "mrad"),
            Cell("Tip position sense resolution", self.tip_sense_resolution * 1e6, "um"),
        ]


class StatusChecks:
    def passes(self, condition, ok_text, fail_text):
        return ok_text if condition else fail_text

    def __init__(self, board, coil, halbach, piece, snap, config, control, sensing, thermal, quiet_thermal, surface, propulsion, attitude, coupled, eddy, stability, drive, tiles, psu, sim):
        self.force = self.passes(config.available_margin >= 1, "OK", "not enough force")
        self.safety = self.passes(config.available_margin >= Inputs.force_safety_factor, "OK", "below safety margin")
        self.voltage = self.passes(config.voltage_per_winding <= config.usable_drive_voltage, "OK", "voltage too high")
        self.baseplate_thermal = self.passes(thermal.cyclic_peak_baseplate_temp <= Inputs.max_surface_temperature, "OK", "baseplate too hot")
        self.source_thermal = self.passes(thermal.cyclic_peak_source_temp <= Inputs.coil_bed_temp_limit, "OK", "coil/MOSFET source above internal material limit")
        self.local_hotspot = self.passes(thermal.worst_piece_local_temp <= Inputs.coil_bed_temp_limit, "OK", "worst-case cell above internal material limit")
        self.magnet_soak = self.passes(thermal.worst_piece_local_temp <= Fixed.magnet_max_operating_temperature, "OK", "resting-piece magnets soak above grade rating")
        self.quiet_baseplate_thermal = self.passes(quiet_thermal.cyclic_peak_baseplate_temp <= Inputs.max_surface_temperature, "OK", "silent-mode baseplate too hot")
        self.quiet_source_thermal = self.passes(quiet_thermal.cyclic_peak_source_temp <= Inputs.coil_bed_temp_limit, "OK", "silent-mode coil/MOSFET source above internal material limit")
        self.quiet_local_hotspot = self.passes(quiet_thermal.worst_piece_local_temp <= Inputs.coil_bed_temp_limit, "OK", "silent-mode cell above internal material limit")
        self.maneuvering = self.passes(propulsion.acceleration_in_g >= Inputs.min_maneuver_accel_g, "OK", "lateral thrust too weak")
        self.tilt_authority = self.passes(attitude.tilt_margin >= 1, "OK", "not enough tilt torque")
        self.yaw_authority = self.passes(attitude.yaw_margin >= 1, "OK", "not enough yaw torque")
        self.worst_force = self.passes(config.worst_available_margin >= Inputs.force_safety_factor, "OK", "worst-case pose lift below safety margin")
        self.showpiece_lift = self.passes(coupled.design_lift_margin >= Inputs.force_safety_factor, "OK", "cannot hold hover at any showpiece tilt rung")
        self.worst_maneuvering = self.passes(propulsion.worst_acceleration_in_g >= Inputs.min_maneuver_accel_g, "OK", "worst-case lateral thrust too weak")
        self.worst_tilt_authority = self.passes(attitude.worst_tilt_margin >= 1, "OK", "worst-case tilt torque too weak")
        self.worst_yaw_authority = self.passes(attitude.worst_yaw_margin >= 1, "OK", "worst-case yaw torque too weak")
        self.coupled_lift = self.passes(coupled.lift_margin >= Inputs.force_safety_factor, "OK", "verified worst-case lift below safety margin")
        self.coupled_maneuvering = self.passes(eddy.slide_margin >= 1, "OK", "worst-pose authority cannot make the 0.5s slide-to-phase")
        self.cruise_authority = self.passes(eddy.cruise_margin >= 1, "OK", "level-pose authority cannot cruise the diagonal in the atomic-cycle window")
        self.coupled_tilt_authority = self.passes(coupled.design_tilt_margin >= 1, "OK", "no showpiece tilt rung is torque-affordable while hovering")
        self.coupled_yaw_authority = self.passes(coupled.yaw_margin >= 1, "OK", "coupled worst-case yaw torque too weak while hovering")
        self.eddy_drag = self.passes(eddy.cruise_margin >= 1 and eddy.slide_margin >= 1, "OK", "eddy drag breaks a flight scenario at its own speed")
        self.rock_controllable = self.passes(stability.control_margin_over_rock >= Inputs.control_loop_bandwidth_margin, "OK", "rock mode too fast for loop")
        self.tilt_observable = self.passes(stability.tip_sense_resolution <= 0.001, "OK", "tilt sensing too coarse")
        self.driver_voltage = self.passes(config.bus_voltage <= Fixed.mosfet_voltage_rating, "OK", "bus exceeds MOSFET voltage rating")
        self.current_offset = self.passes(drive.current_offset_error <= Fixed.max_current_offset_fraction * Fixed.driver_channel_current, "OK", "comparator offset degrades current accuracy")
        self.midpoint_balance = self.passes(psu.rail_imbalance_current <= psu.unit_rated_current, "OK", "rail imbalance exceeds PSU unit current rating")
        self.zone_capacity = self.passes(psu.zone_hover_piece_capacity >= psu.zone_required_pieces, "OK", "one zone cannot hover its half of the reset formation")
        self.visible_hover = self.passes(surface.visible_hover >= Inputs.min_visible_hover_height, "OK", "top stack eats the visible hover height")
        self.surface_flatness = self.passes(surface.flatness_fraction <= 0.25, "OK", "flatness budget too large vs visible hover")
        self.hotspot_touch = self.passes(max(thermal.worst_piece_local_temp, quiet_thermal.worst_piece_local_temp) <= Fixed.max_touch_temperature, "OK", "worst cell exceeds brief-touch limit; a hand-lifted piece could expose a burning square")
        self.prolonged_touch = self.passes(surface.idle_touch_temperature <= Fixed.prolonged_touch_temperature, "OK", "idle surface exceeds prolonged-contact touch limit")
        self.shunt_power = self.passes(config.worst_required_current ** 2 * Fixed.current_sense_resistance <= 0.5 * Fixed.current_shunt_power_rating, "OK", "shunt dissipation exceeds derated rating at worst-case current")
        self.driver_current = self.passes(config.worst_required_current <= Fixed.driver_channel_current, "OK", "required coil current exceeds channel rating")
        self.actuator_rank = self.passes(sim["actuator_rank6"] >= 6, "OK", "actuator matrix not full 6-DOF rank")
        self.hall_observable = self.passes(sensing.worst_rank >= 6, "OK", "Hall array cannot observe all 6 DOF across fixed-grid phases")
        self.hall_condition = self.passes(sensing.worst_condition <= 2 ** Fixed.hall_interpolation_bits, "OK", "Hall worst-case observability condition too high for ADC resolution")
        self.hall_saturation = self.passes(sensing.saturation_field <= Fixed.hall_linear_range, "OK", "Hall sensors saturate under worst-case magnet + coil field")
        self.shell_validity = self.passes((piece.diameter - 2 * Inputs.plastic_wall_thickness) > 0, "OK", "wall too thick")
        self.flight_gap = self.passes(Inputs.max_flight_gap >= Inputs.magnet_to_coil_distance, "OK", "maximum flight gap below resting gap")
        self.coil_height = self.passes(config.coil_height <= coil.outer_width, "OK", "coil too tall")
        self.coil_window = self.passes(config.inner_window_width > 0 and config.inner_window_length > 0, "OK", "winding walls overlap, no coil opening")
        self.platform_size = self.passes(20 <= board.platform_side <= 50, "OK", "platform out of range")
        self.chess_square_size = self.passes(board.square_size <= Fixed.max_chess_square_size, "OK", "chess square too large")
        self.square_alignment = self.passes(
            abs(board.square_size / board.period_length - round(board.square_size / board.period_length)) < 1e-9
            and abs(board.square_size / coil.outer_length - round(board.square_size / coil.outer_length)) < 1e-9
            and abs(board.square_size / coil.outer_width - round(board.square_size / coil.outer_width)) < 1e-9,
            "OK", "square pitch not an integer multiple of magnet and coil lattices")
        self.magnet_fits_base = self.passes(board.platform_side <= board.base_diameter, "OK", "magnet array wider than base")
        self.neighbour_snap = self.passes(snap.snap_to_weight <= snap.holding_friction, "OK", "resting pieces magnetically snap")
        self.control_bandwidth = self.passes(control.actuator_bandwidth >= control.required_bandwidth, "OK", "actuator bandwidth too low")
        self.current_slew = self.passes(control.slew_time <= control.instability_time / Inputs.control_loop_bandwidth_margin, "OK", "current cannot react in time")
        self.hall_throughput = self.passes(sensing.headroom >= 1, "OK", "tile ADC too slow to scan Hall grid")
        self.hall_resolution = self.passes(sensing.total_position_error_um <= sensing.required_position_error_um, "OK", "Hall position error exceeds required resolution")
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
        self.burst_coverage = self.passes(
            psu.buffer.usable_energy >= psu.buffer_energy and psu.buffer.peak_power >= psu.burst_deficit_power,
            "OK", "supercap buffer undersized for reset burst")
        self.buffer_recharge = self.passes(psu.recharge_time <= psu.recharge_window, "OK", "buffer cannot recharge before the next reset")
        self.bus_current = self.passes(psu.required_current <= Fixed.max_bus_current, "OK", "bus current too high")

    def cells(self):
        return [
            Cell("Force check", self.force),
            Cell("Safety-margin check", self.safety),
            Cell("Voltage check", self.voltage),
            Cell("Baseplate thermal check", self.baseplate_thermal),
            Cell("Coil/MOSFET thermal check", self.source_thermal),
            Cell("Worst-case cell hotspot check", self.local_hotspot),
            Cell("Magnet soak-temperature check", self.magnet_soak),
            Cell("Silent-mode baseplate check", self.quiet_baseplate_thermal),
            Cell("Silent-mode source check", self.quiet_source_thermal),
            Cell("Silent-mode cell hotspot check", self.quiet_local_hotspot),
            Cell("Maneuvering check", self.maneuvering),
            Cell("Tilt-authority check", self.tilt_authority),
            Cell("Yaw-authority check", self.yaw_authority),
            Cell("Worst-case-pose lift check", self.worst_force),
            Cell("Showpiece-tilt lift check", self.showpiece_lift),
            Cell("Worst-case maneuvering check", self.worst_maneuvering),
            Cell("Worst-case tilt-authority check", self.worst_tilt_authority),
            Cell("Worst-case yaw-authority check", self.worst_yaw_authority),
            Cell("Verified worst-case lift check", self.coupled_lift),
            Cell("Coupled worst-case maneuvering check", self.coupled_maneuvering),
            Cell("Cruise-authority check", self.cruise_authority),
            Cell("Coupled worst-case tilt-authority check", self.coupled_tilt_authority),
            Cell("Coupled worst-case yaw-authority check", self.coupled_yaw_authority),
            Cell("Eddy-drag check", self.eddy_drag),
            Cell("Rock-mode controllable check", self.rock_controllable),
            Cell("Tilt-observable check", self.tilt_observable),
            Cell("Driver-bus-voltage check", self.driver_voltage),
            Cell("Current-offset check", self.current_offset),
            Cell("Split-rail midpoint-balance check", self.midpoint_balance),
            Cell("PSU zone-capacity check", self.zone_capacity),
            Cell("Visible-hover-height check", self.visible_hover),
            Cell("Surface-flatness check", self.surface_flatness),
            Cell("Hotspot-touch-temperature check", self.hotspot_touch),
            Cell("Prolonged-touch-temperature check", self.prolonged_touch),
            Cell("Shunt-dissipation check", self.shunt_power),
            Cell("Driver-channel-current check", self.driver_current),
            Cell("Actuator 6-DOF rank check", self.actuator_rank),
            Cell("Hall 6-DOF observability check", self.hall_observable),
            Cell("Hall observability-condition check", self.hall_condition),
            Cell("Hall-saturation check", self.hall_saturation),
            Cell("Shell-validity check", self.shell_validity),
            Cell("Flight-gap-range check", self.flight_gap),
            Cell("Coil-height buildable check", self.coil_height),
            Cell("Coil-window non-degenerate check", self.coil_window),
            Cell("Platform-size check", self.platform_size),
            Cell("Chess-square-size check", self.chess_square_size),
            Cell("Square-lattice phase-alignment check", self.square_alignment),
            Cell("Magnet-array-fits-base check", self.magnet_fits_base),
            Cell("Neighbour-snap check", self.neighbour_snap),
            Cell("Control-bandwidth check", self.control_bandwidth),
            Cell("Current-slew check", self.current_slew),
            Cell("Hall-throughput check", self.hall_throughput),
            Cell("Hall-position-error check", self.hall_resolution),
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
            Cell("Reset-burst coverage check", self.burst_coverage),
            Cell("Buffer-recharge check", self.buffer_recharge),
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
    def __init__(self, board, wire, piece, thermal, psu):
        self.board_copper_mass = wire.copper_mass
        self.board_pcb_mass = board.motor_area * Fixed.pcb_thickness * Constants.fr4_density / 1000
        self.radiator_mass = thermal.aluminium_mass
        self.cooling_fan_mass = thermal.fan_mass
        self.buffer_mass = psu.buffer.mass
        self.gap_filler_mass = board.motor_area * Fixed.radiator_standoff_below_pcb / 1000 * Fixed.gap_filler_density / 1000
        self.board_added_mass = psu.unit_count * Fixed.psu_unit_mass_kg + Fixed.frame_enclosure_mass_kg + Fixed.board_electronics_mass_kg
        self.board_total_mass = (self.board_copper_mass + self.board_pcb_mass + self.radiator_mass + self.cooling_fan_mass
                                 + self.buffer_mass + self.gap_filler_mass + self.board_added_mass)
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
            Cell("Supercap burst buffer", self.buffer_mass, "kg"),
            Cell("Dispensed gap filler", self.gap_filler_mass, "kg"),
            Cell("PSU + frame + electronics (est.)", self.board_added_mass, "kg"),
            Cell("Board total (est.)", self.board_total_mass, "kg"),
            Cell("Mass per piece", self.piece_mass * 1000, "g"),
            Cell("Pieces total", self.pieces_total),
            Cell("All pieces mass", self.all_pieces_mass, "kg"),
            Cell("WHOLE SET mass (est.)", self.set_total_mass, "kg"),
        ]


class BillOfMaterials:
    def __init__(self, board, coil, halbach, wire, config, tiles, sensing, thermal, psu):
        coils_per_tile = ceil(coil.total_bodies / tiles.tile_count)
        half_bridges_per_tile = coils_per_tile * coil.half_bridges_per_coil
        tile_power_mosfets = half_bridges_per_tile
        tile_gate_drivers = ceil(half_bridges_per_tile / Fixed.gate_driver_half_bridges)
        tile_shift_registers = ceil(coils_per_tile / Fixed.shift_register_outputs)
        tile_setpoint_filter_pairs = coils_per_tile
        tile_current_shunts = coils_per_tile
        tile_current_comparators = ceil(coils_per_tile / Fixed.current_comparator_channels_per_ic)
        tile_current_frontend_passives = coils_per_tile * Fixed.current_frontend_passives_per_channel
        tile_driver_passives = half_bridges_per_tile * 2
        tile_driver_decoupling = tile_shift_registers + tile_gate_drivers + tile_current_comparators
        tile_driver_solder_joints = (
            tile_power_mosfets * 8
            + tile_gate_drivers * 20
            + tile_shift_registers * 16
            + tile_setpoint_filter_pairs * 4
            + tile_current_shunts * 2
            + tile_current_comparators * 8
            + tile_current_frontend_passives * 2
            + tile_driver_passives * 2
            + tile_driver_decoupling * 2
            + Fixed.tile_bulk_capacitors_per_tile * 2
            + Fixed.setpoint_fpga_solder_joints
        )
        tile_hall_sensors = sensing.sensors_per_tile
        tile_hall_muxes = sensing.muxes_per_tile
        tile_wire_kg = wire.copper_mass / tiles.tile_count
        tile_pcb_area_cm2 = (tiles.tile_side ** 2) / 100
        gap_filler_volume_cc = board.motor_area * Fixed.radiator_standoff_below_pcb / 1000

        self.tile_count = tiles.tile_count
        self.piece_count = Fixed.captured_pieces_total
        self.coils_per_tile = coils_per_tile

        self.tile_items = [
            BomItem("tile", "Driver power MOSFET", "40V dual N-MOSFET SOP-8/PDFN, one half-bridge/package; price is RFQ target", tile_power_mosfets, Fixed.power_mosfet_price, "https://www.lcsc.com/product-detail/C20539695.html"),
            BomItem("tile", "Driver gate driver", "EG Micro EG2134 3 half-bridge MOSFET driver (LCSC C480661)", tile_gate_drivers, Fixed.gate_driver_price, "https://www.lcsc.com/product-detail/C480661.html"),
            BomItem("tile", "Current setpoint latch", "Gcore GR74HC595 8-bit shift register (LCSC C18164493)", tile_shift_registers, Fixed.shift_register_price, "https://www.lcsc.com/product-detail/C18164493.html"),
            BomItem("tile", "Setpoint RC filter", "15.8k 1% + 10nF X7R 0603 pair, 1.007kHz (LCSC C155689 + C519406)", tile_setpoint_filter_pairs, Fixed.setpoint_filter_passive_price, "https://www.lcsc.com/product-detail/C519406.html"),
            BomItem("tile", "Setpoint FPGA", "GOWIN GW1NZ-LV1QN48C6 + 1.2V LDO; BRAM-multiplexed delta-sigma fabric, synthesis proof pending", 1, Fixed.setpoint_fpga_price, "https://www.lcsc.com/product-detail/C5799569.html"),
            BomItem("tile", "Current shunt", "Milliohm HoJLR2512-2W-20mR-1% 75ppm midpoint-return sense; full-reel RFQ pending", tile_current_shunts, Fixed.current_shunt_price, "https://www.lcsc.com/product-detail/C2924538.html"),
            BomItem("tile", "Current comparator", "MSKSEMI LM393 dual comparator (LCSC C5252905)", tile_current_comparators, Fixed.current_comparator_price, "https://www.lcsc.com/product-detail/C5252905.html"),
            BomItem("tile", "Current front-end passives", "0603 1% + matched-pair arrays (idle zero-cal removes statics) [TO BE SOURCED]", tile_current_frontend_passives, Fixed.current_frontend_passive_price, ""),
            BomItem("tile", "Tile bulk capacitance", "330-470uF 16V polymer local power decoupling for the 20kHz half-bridge bank [TO BE SOURCED]", Fixed.tile_bulk_capacitors_per_tile, Fixed.tile_bulk_capacitor_price, ""),
            BomItem("tile", "Driver gate passives", "0603 1% gate pull resistors (LCSC C54531144 class)", tile_driver_passives, Fixed.driver_gate_passive_price, "https://www.lcsc.com/product-detail/C54531144.html"),
            BomItem("tile", "Driver decoupling", "100nF 50V X7R 0603 logic bypass capacitors (LCSC C14663 class)", tile_driver_decoupling, Fixed.driver_decoupling_price, "https://www.lcsc.com/product-detail/C14663.html"),
            BomItem("tile", "Driver SMT assembly", "JLCPCB automated assembly joints", tile_driver_solder_joints, Fixed.smt_assembly_cost_per_joint, "https://jlcpcb.com/help/article/pcb-assembly-faqs"),
            BomItem("tile", "Magnet wire", f"{config.wire.label} rectangular self-bonding enameled copper wire (kg share; price is RFQ budget)", tile_wire_kg, 18.74, "https://enameledwires.com/products/enameled-copper-wire/self-bonding-rectangular.html"),
            BomItem("tile", "Hall position sensor", "TI DRV5055A4QDBZR, 12.5 V/T, +/-169mT, approx. 18.4uT rms typical input noise", tile_hall_sensors, Fixed.hall_sensor_price, "https://www.digikey.com/en/products/detail/texas-instruments/DRV5055A4QDBZR/8567410"),
            BomItem("tile", "Hall group gate switch", "GOODWORK AO3401A -30V P-FET, one per mux group (LCSC C2938368)", tiles.hall_muxes_per_tile, Fixed.hall_gate_switch_price, "https://www.lcsc.com/product-detail/MOSFETs_GOODWORK-AO3401A_C2938368.html"),
            BomItem("tile", "Hall readout mux", "TI CD74HC4067SM96 16ch analog mux (LCSC C98457)", tile_hall_muxes, Fixed.hall_sensor_mux_price, "https://www.lcsc.com/product-detail/C98457.html"),
            BomItem("tile", "Tile PCB", "JLCPCB 4-layer FR4 100x100mm, 100-board quote equivalent", tile_pcb_area_cm2, Fixed.tile_pcb_price_per_cm2, "https://jlcpcb.com/news/discount-on-quality-4-layer-pcbs"),
            BomItem("tile", "Tile control MCU", "STM32G431KBT6 32-pin", 1, 3.13, "https://www.digikey.com/en/products/detail/stmicroelectronics/STM32G431KBT6/10231564"),
            BomItem("tile", "Backplane connector", "ZHOURI 2x10 2.54mm male header (LCSC C5116480)", 1, Fixed.tile_connector_header_price, "https://www.lcsc.com/product-detail/C5116480.html"),
        ]
        self.piece_items = [
            BomItem("piece", "NdFeB magnet block", f"N48SH {Inputs.magnet_lateral_edge:g}x{Inputs.magnet_lateral_edge:g}x{Inputs.magnet_thickness:g}mm cube (SH grade for hot-cell soak) [TO BE SOURCED]", halbach.blocks_per_platform, halbach.block_mass / 1000 * Fixed.magnet_cost_per_kg, "https://www.jc-magnetics.com/Magnet-N52-5mmx5mmx5mm-Cube"),
            BomItem("piece", "Piece plastic / misc", "PLA print material plus inserts/finish allowance", 1, 1.4, "https://jlc3dp.com/blog/3d-printing-cost"),
        ]
        self.board_items = [
            BomItem("board", "Compute module", "RPi CM5 2GB Lite, SC1556 (57.37 EUR)", 1, 61.96, "https://www.digikey.com/en/products/detail/raspberry-pi/SC1556/25805567"),
            BomItem("board", "Mainboard", "Custom 4-layer carrier, priced from JLCPCB quote basis", 1, 25.0, "https://jlcpcb.com/quote"),
            BomItem("board", "Tile interconnect", "HDGC 2x10 2.54mm female socket (LCSC C19725277)", tiles.tile_count, Fixed.tile_connector_socket_price, "https://www.lcsc.com/product-detail/C19725277.html"),
            BomItem("board", "Bus power supply", psu.psu_family, psu.unit_count, psu.unit_price, psu.psu_url),
            BomItem("board", "Bus distribution", "Copper 110 flat busbar plus zone cabling allowance", 1, Fixed.bus_distribution_price, "https://www.ebay.com/itm/304578689563"),
            BomItem("board", "Rail regen clamp", "TVS + dump resistor per rail per zone; PSUs cannot sink braking energy [TO BE SOURCED]", 2 * psu.zones, Fixed.rail_clamp_price, ""),
            BomItem("board", "Rail bulk capacitance", "Low-ESR bulk electrolytic per rail per zone [TO BE SOURCED]", 2 * psu.zones, Fixed.rail_bulk_capacitor_price, ""),
            BomItem("board", "Supercap burst buffer", f"Maxwell BCAP0350-P270-S18, {psu.buffer.series_cells}s{psu.buffer.strings_per_rail}p per rail; allocation advised", psu.buffer.cell_count, Fixed.supercap_cell_price, "https://www.digikey.com/en/products/detail/maxwell-technologies/BCAP0350-P270-S18/11673891"),
            BomItem("board", "Supercap balancing", "active balancing network per cell group; topology decision pending [TO BE SOURCED]", psu.buffer.cell_count, Fixed.supercap_balancer_price_per_cell, ""),
            BomItem("board", "Buffer charge/protection", "BQ33100-class monitor + precharge/charge path + fuse + disconnect per rail bank [TO BE SOURCED]", 2, Fixed.supercap_management_price_per_rail, "https://www.ti.com/product/BQ33100"),
            BomItem("board", "Buffer ideal-diode ORing", "LM74800-Q1 + paralleled 40V N-FET pairs per rail per zone; 103A path thermal validation pending [TO BE SOURCED]", psu.buffer.oring_count, Fixed.supercap_oring_price, "https://www.ti.com/lit/ds/symlink/lm7480.pdf"),
            BomItem("board", "Radiator aluminium", "Integral-fin extrusion, crosshatch-kerfed 4mm base (fins below the web, outside the eddy field); RFQ budget per kg", thermal.aluminium_mass, Fixed.radiator_aluminium_price_per_kg, ""),
            BomItem("board", "Radiator eddy-break slotting", "Gang-saw/CNC 5mm-pitch crosshatch through 3.5mm of extrusion base; RFQ budget", 1, Fixed.radiator_slotting_price, ""),
            BomItem("board", "Coil potting epoxy", "Ziitek TIE280-25AB-class epoxy, 2.5 W/mK; price remains RFQ budget", 1, Fixed.potting_epoxy_price, "https://www.ziitek.com/epoxy-potting-compound"),
            BomItem("board", "Dispensed thermal gap filler", f"Laird Tputty SF560 5.6 W/mK, 1.5mm bond line, {gap_filler_volume_cc:.1f}cc; 10-pail public price, RFQ + selective dispensing pending", 1, Fixed.gap_filler_pad_price, "https://www.laird.com/products/thermal-interface-materials/liquid-gap-fillers/tputty-sf560"),
            BomItem("board", "Playing surface", "UV-printed board graphics + clear wear topcoat applied directly on the potting [TO BE SOURCED]", 1, Fixed.playing_surface_price, ""),
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
sim_geometry = levitation_sim.SimGeometry(
    magnet_lateral_edge_mm=Inputs.magnet_lateral_edge,
    magnet_thickness_mm=Inputs.magnet_thickness,
    magnets_per_period=Inputs.magnets_per_period,
    periods_per_side=Inputs.periods_per_side,
    magnet_to_coil_distance_mm=Inputs.magnet_to_coil_distance,
    max_flight_gap_mm=Inputs.max_flight_gap,
    plastic_wall_thickness_mm=Inputs.plastic_wall_thickness,
    base_corner_standoff_mm=Fixed.base_corner_standoff,
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
    surface_stack_mm=Fixed.potting_cover_thickness + Fixed.playing_surface_thickness + Fixed.piece_bottom_skin,
    tilt_rim_clearance_mm=Inputs.tilt_rim_clearance,
    pcb_thickness_mm=Fixed.pcb_thickness,
    hall_package_standoff_mm=Fixed.hall_package_standoff,
)
sim = levitation_sim.measure(sim_geometry)
halbach = HalbachArray(board, sim)
piece = Piece(board, halbach)
snap = NeighbourSnap(board, piece, halbach, sim)
sweep = ConfigurationSweep(board, coil, halbach, piece, sim)
config = sweep.selected
coil.outer_height = config.coil_height
wire = WireThermal(coil, config)
driver = DiscreteDriver(board, coil, config)
passive_thermal = RadiatorCooling(board, config, driver, 0, False)
thermal = RadiatorCooling(board, config, driver, Inputs.active_cooling_fans, True)
propulsion = Propulsion(board, coil, piece, config, halbach, sim)
attitude = AttitudeAuthority(board, piece, config, sim)
coupled = CoupledAuthority(coil, piece, config, attitude)
eddy = EddyDrag(board, config, piece, coupled)
control = Control(coil, config, sim)


def select_hall_pitch():
    for pitch in Fixed.hall_pitch_candidates:
        trial_tiles = TileControl(board, coil, control, pitch)
        trial = HallSensing(coil, config, control, trial_tiles)
        tip_resolution = trial.tilt_noise_mrad / 1000 * (piece.box_height / 1000)
        if (trial.worst_rank == 6
                and trial.worst_condition <= 2 ** Fixed.hall_interpolation_bits
                and trial.total_position_error_um <= trial.required_position_error_um
                and trial.saturation_field <= Fixed.hall_linear_range
                and trial.headroom >= 1
                and tip_resolution <= 0.001):
            return trial_tiles, trial
    return trial_tiles, trial


tiles, sensing = select_hall_pitch()
drive = DriveMatrix(coil, control, driver, tiles)
psu = PowerSupply(coil, wire, tiles, config, driver, thermal)
stability = Stability(board, piece, control, sensing, sim)
surface = SurfaceStack(thermal)
checks = StatusChecks(board, coil, halbach, piece, snap, config, control, sensing, thermal, passive_thermal, surface, propulsion, attitude, coupled, eddy, stability, drive, tiles, psu, sim)
bom = BillOfMaterials(board, coil, halbach, wire, config, tiles, sensing, thermal, psu)
mass = MassBudget(board, wire, piece, thermal, psu)


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
    title = "Configuration sweep (lowest-power feasible coil per bus voltage)"
    print(title)
    print("-" * len(title))
    print(f"  {'bus V':>6}{'wire':>16}{'layers':>7}{'turns':>7}{'op mA':>8}{'source C':>9}{'avail x':>9}")
    for entry in sweep.best_per_voltage:
        marker = "  <- selected" if entry is sweep.selected else ""
        entry_driver = DiscreteDriver(board, coil, entry)
        source_temp = RadiatorCooling(board, entry, entry_driver, Inputs.active_cooling_fans, True).cyclic_peak_source_temp
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
    print_section("Silent-mode cooling (fans off, live play)", passive_thermal.cells())
    print_section("Playing surface stack", surface.cells())
    if Inputs.active_cooling_fans:
        print_section("Spectate-mode cooling (fans on, real-time replay)", thermal.cells())
    print_section("Propulsion / flight", propulsion.cells())
    print_section("Attitude authority (tilt / yaw)", attitude.cells())
    print_section("Verified worst-case authority (real coil geometry)", coupled.cells())
    print_section("Eddy-current drag (conductive sheets)", eddy.cells())
    print_section("Control feasibility", control.cells())
    print_section("Drive matrix (position-addressed)", drive.cells())
    print_section("Hall position sensing", sensing.cells())
    print_section("Tiled control architecture", tiles.cells())
    print_section("Power supply", psu.cells())
    print_section("Burst energy buffer (supercaps)", psu.buffer.cells())
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
