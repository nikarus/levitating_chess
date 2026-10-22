class Inputs:
    # Requirements and design/search choices; provisional assumptions are identified below.
    magnet_lateral_edge = 5                     # mm; square permanent-magnet block edge in the Halbach array
    magnet_thickness = 4                        # mm; baseline block thickness, not a newly optimized value
    magnets_per_period = 4                      # blocks per spatial magnetic period
    periods_per_side = 1                        # magnetic periods along each side of the piece array
    coils_per_period = 2                        # coil pitches per spatial magnetic period
    coil_outer_length = 15                      # mm; long outside dimension of each rectangular coil
    winding_radial_width = 1.0                  # mm; winding wall available to copper, insulation and clearance
    control_cells_per_side = 4                  # coil-pitch cells along each side of the local actuation window
    magnet_to_coil_distance = 3.5               # mm; nominal magnet-bottom to top-coil gap in flight
    max_flight_gap = 4.5                        # mm; upper magnet-to-coil gap sampled for flight authority and sensing
    plastic_wall_thickness = 1.0                # mm; hollow piece-shell wall used in the mass estimate
    piece_shell_shape_factor = 0.6              # ratio of hollow king bounding-cylinder shell mass/inertia retained uniformly; provisional silhouette approximation
    worst_phase_dwell = 0.5                     # s; flight takeoff dwell, charged at worst sampled hover power
    cruise_duration = 1.5                       # s (atomic move A: up to full-diagonal flight in this window)
    selected_bus_voltage = 40                   # V; retained full-bus baseline with a controlled midpoint; not re-optimized
    landing_dwell = 0.5                         # s (aligned-slot approach and settle)
    events_back_to_back = 2                     # count (immediate-rematch resets with no cooldown)
    sustained_event_period = 300                # s (one full reset event sustained forever)
    play_move_period = 6                        # s per composite move in the passive-cooling sustained workload
    sustained_moves_per_minute = 10             # composite moves/min in the fan-assisted workload; user-approved sustained target, checked against serial move timing
    replay_capture_fraction = 0.25              # ratio (replay moves that add a second storage flight)
    replay_knight_gap_fraction = 0.15           # ratio (replay moves needing blockers to part and re-close)
    hammer_visit_period = 5                     # s; T3 repeat period for a dwell on the same cell
    hammer_dwell = 1.0                          # s; T3 aligned hover dwell per visit
    cascade_exchanges = 10                      # exchanges in the T3 burst; each contributes two local dwells
    coil_bed_temp_limit = 105                   # C; internal winding/potting acceptance limit; touch limits are checked separately
    force_safety_factor = 1.3                   # ratio; required lift reserve above piece weight
    min_maneuver_accel_g = 0.2                  # g; minimum horizontal flight acceleration, also checked against travel timing
    min_visible_hover_height = 3                # mm (piece bottom to playing surface at nominal hover)
    min_level_hover_endurance = 30              # s; retained continuous parked-hover requirement; no thermal-governor credit
    min_showpiece_hover_endurance = 3           # s; retained deepest-tilt hover requirement on one cell
    tilt_rim_clearance = 1.0                    # mm (tilt limited so the lowest rim point keeps this clearance above the playing surface)
    target_tilt_time = 0.3                      # s
    target_yaw_angle_deg = 90                   # deg
    target_yaw_time = 0.5                       # s
    ambient_temperature = 35                    # C
    max_surface_temperature = 50                # C; baseplate-temperature acceptance limit in the current thermal screen
    control_loop_bandwidth_margin = 5           # ratio; heuristic multiplier on nominal unstable growth rate, not a closed-loop stability proof
    pieces_levitating_simultaneously = 1        # airborne pieces in normal play; independent of the reset population
    drive_look_ahead_factor = 1.5               # ratio; extra driven-channel allowance around the active flight footprint
    active_cooling_fans = 2                     # fans in the fan-assisted thermal scenario; passive scenario uses no fans
    reset_duration = 8.0                        # s; provisional full-set contact-reset duration, not a routing guarantee
    reset_breakaway_duration = 0.5              # s; provisional static-friction phase within reset_duration
    production_boards = 100                     # boards per production order used to label the BOM cost assumptions
    piece_count = 32                            # pieces in the full set, contact reset and total storage capacity
    max_tip_position_error = 0.001              # m; maximum lateral tip error inferred from Hall tilt uncertainty
    max_surface_flatness_fraction = 0.25        # ratio; allowed surface flatness budget divided by visible hover height
    platform_side_limits = (20, 50)             # mm; accepted lower and upper bounds for the square magnetic platform
    trajectory_force_margin = 1.05              # ratio; acceleration reserve on the full-diagonal triangular motion profile
    selected_conductor_thickness = 0.05         # mm; adopted bare-copper thickness from the winding screen; supplier confirmation outstanding
    selected_winding_layers = 57                # axial layers; adopted prototype winding, with turns per layer defined in Fixed
    standard_bus_voltages = (24, 30, 40, 48)    # V; bus candidates for the preliminary winding screen
    winding_conductor_thicknesses = (selected_conductor_thickness, 0.07, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5) # mm; bare rectangular-copper thickness candidates; supplier availability unverified
    coil_layout_candidates = (
        ('stacked_reference', 'stacked', magnets_per_period * magnet_lateral_edge / coils_per_period, coil_outer_length, winding_radial_width),
        ('parquet_5', 'herringbone', 5, 10, 1),
        ('parquet_7_5', 'herringbone', 7.5, 15, 1),
        ('parquet_10', 'herringbone', 10, 20, 1),
        ('parquet_wide_wall', 'herringbone', 7.5, 15, 2),
        ('square_7_5', 'square', 7.5, 7.5, 1),
        ('square_10', 'square', 10, 10, 1),
        ('square_12_5', 'square', 12.5, 12.5, 1),
        ('square_wide_wall', 'square', 10, 10, 2),
    )                                          # exploratory layouts: name, pattern, short/long outer dimensions and winding wall in mm; not adopted hardware
    prototype_layouts = (('parquet_5', 0), ('square_7_5', 45)) # shortlisted footprint names and lattice rotations in degrees
    prototype_wire_diameter = 0.25              # mm; nominal bare round copper, Elektrisola IEC technical-data row
    prototype_radial_turns = 4                  # turns across the winding wall; rectangular round-wire packing assumption
    prototype_axial_layers = 14                 # even layer count brings both winding ends to the inner window; 56 turns total
    prototype_inner_corner_radius = 1.0         # mm; proposed winding-window radius; continuous conductor bends still need virtual detailing
    prototype_coil_spacing = 0.3                # mm; minimum nominal assembly gap between winding envelopes
    cad_coil_layout = 'square_7_5'              # candidate shown in the integrated virtual assembly; main hardware targets remain under review
    channel_layout_candidates = (
        ('square_reference', 'square_7_5', 45, prototype_coil_spacing, 'independent'),
        ('square_spaced', 'square_7_5', 45, 1.5, 'independent'),
        ('square_larger', 'square_10', 45, prototype_coil_spacing, 'independent'),
        ('square_largest', 'square_12_5', 45, prototype_coil_spacing, 'independent'),
        ('parquet_reference', 'parquet_5', 0, prototype_coil_spacing, 'independent'),
        ('parquet_larger', 'parquet_7_5', 0, prototype_coil_spacing, 'independent'),
        ('paired_same', 'square_7_5', 45, prototype_coil_spacing, 'series_same'),
        ('paired_opposed', 'square_7_5', 45, prototype_coil_spacing, 'series_opposed'),
    )                                          # study name, existing geometry, lattice angle (deg), envelope spacing (mm), local driver connection; not adopted hardware
    channel_comparison_candidates = ('square_reference', 'square_larger') # retained independent-coil designs for the broader static comparison
    channel_sweep_square_sizes = (7.5, 8.25, 9., 10., 12.5) # mm; square outside dimensions for the compact geometry sweep
    channel_sweep_parquet_sizes = (5., 6.25, 7.5, 10.) # mm; short outside dimension, with the implemented two-to-one parquet rectangle
    channel_sweep_spacings = (prototype_coil_spacing, 0.75, 1.5) # mm; gaps along the short lattice axis; parquet long-axis gaps are doubled
    channel_sweep_connections = ('independent', 'series_same', 'series_opposed') # square driver branches; parquet currently supports independent coils only
    electronics_full_bridge_buses = (24., selected_bus_voltage) # V; single-supply bridge candidates, retaining the existing simulated terminal-voltage limit
    electronics_placement_cells_per_side = 4    # coils per tile edge; density-study window aligned with the existing coil lattice, not a selected production module


class Fixed:
    # Implementation, numerical and costing assumptions, not immutable physical constants.
    # Historical component specifications and prices require validation before hardware selection.
    base_corner_standoff = 8                    # mm
    square_fill_ratio = 0.8                     # ratio (maximum; actual fill falls out of period snapping)
    max_chess_square_size = 60                  # mm
    resting_friction_coefficient = 0.4          # ratio; assumed static friction for reset breakaway and powered-off no-snap checks
    captured_side_areas = 2                     # motorized storage areas flanking the playing board
    herringbone_orientation_families = 2        # count
    reference_king_height = 95                  # mm
    reference_king_base_diameter = 44           # mm
    rectangular_wire_film = 0.012               # mm per face; assumed enamel thickness, to be confirmed with the wire supplier
    turns_per_radial_layer = 1                  # count
    nominal_coil_height_for_field = 1.0         # mm; reference winding height for the preliminary field/coupling screen
    potting_thickness = 0.5                     # mm (dispensed epoxy bed between coil bottoms and PCB; thinned for the hot-copper local stack)
    potting_thermal_conductivity = 2.5          # W/(m.K) Appli-Thane 7300 class thermal potting
    potting_volumetric_heat_capacity = 2300000.0 # J/(m3.K) (volume-weighted: ~21% copper walls + filled-epoxy windows)
    coil_bed_through_conductivity = 2.0         # W/(m.K) (through-plane: enamel-film wire stack in parallel with potted coil windows)
    potting_cover_thickness = 0.2               # mm (self-leveling skim over coil tops)
    playing_surface_thickness = 0.1             # mm (UV-printed graphics + clear wear topcoat directly on the potting; no separate sheet)
    playing_surface_conductivity = 0.2          # W/(m.K) epoxy topcoat
    piece_bottom_skin = 0.1                     # mm (plated magnets flush, thin conformal coat)
    surface_flatness_budget = 0.3               # mm (over full board span)
    max_touch_temperature = 77                  # C; retained brief-touch screening limit applied to every cell, even when covered; material/contact validation outstanding
    prolonged_touch_temperature = 48            # C; retained prolonged-touch screening limit for idle surface regions
    magnet_max_operating_temperature = 150      # C (SH-grade NdFeB rating; resting piece soaks toward the cell temperature)
    pcb_via_effective_thermal_conductivity = 10 # W/(m.K) (dense 0.3mm thermal-via array under the coil bed, ~1mm pitch)
    thermal_pad_conductivity = 5.0              # W/(m.K) premium dispensed gap filler
    baseplate_thickness = 4.0                   # mm
    aluminium_thermal_conductivity = 167        # W/(m.K)
    aluminium_density = 2700                    # kg/m3
    aluminium_heat_capacity = 900               # J/(kg.K)
    fin_height = 60                             # mm; retained heatsink geometry, not resized for the new motion scenarios
    fin_thickness = 2                           # mm
    fin_channel_width = 8                       # mm
    natural_convection_coefficient = 2          # W/(m2.K) (derated: fins face down under an elevated board)
    forced_convection_coefficient = 8           # W/(m2.K)
    cooling_fan_size = 200                      # mm
    cooling_fan_speed = 550                     # rpm
    cooling_fan_airflow = 100.8                 # m3/h
    cooling_fan_static_pressure = 0.51          # mm H2O
    cooling_fan_noise = 10.7                    # dB(A)
    cooling_fan_installation_noise = 3          # dB(A)
    cooling_fan_airflow_fraction = 0.25         # ratio
    cooling_fan_power = 0.96                    # W
    cooling_fan_mass = 0.37                     # kg
    cooling_fan_price = 39.95                   # USD
    cooling_fan_url = 'https://www.coolerguys.com/products/noctua-nf-a20-pwm-200mm-cooling-fan' # historical fan listing used for the BOM; not a refreshed quotation
    mosfet_voltage_rating = 60                  # V
    min_mosfet_voltage_headroom = 1.5           # ratio; MOSFET voltage rating divided by candidate bus voltage must meet this value
    driver_channel_current = 5.5                # A per coil; regulated cycle-average capability, distinct from instantaneous device/protection rating
    driver_peak_current = 6.5                   # A per coil; provisional instantaneous power-stage/protection capability including switching ripple
    driver_max_ripple_current = 0.5             # A; allowed peak deviation from cycle-average current in the split-rail PWM screen
    driver_pwm_frequency = 160000               # Hz; provisional carrier target set by the low-inductance ripple screen, not pose control
    electronics_supply_tolerance = 0.05         # fraction; provisional supply excursion for topology screening, not a qualified PSU specification
    electronics_pwm_screen_points = 101         # mean-voltage samples over each polarity for the independent winding switching screen
    electronics_spice_cycles = 8                # PWM ripple periods per independently solved circuit check, initialized at periodic equilibrium
    electronics_spice_steps_per_cycle = 512     # maximum integration step denominator for the ngspice reference, repeated at twice this resolution
    electronics_spice_edge_fraction = 0.00001   # rise/fall duration divided by ripple period for the ideal voltage-source reference; not a MOSFET switching-time claim
    electronics_spice_current_tolerance = 0.0001 # A; maximum analytic/SPICE ripple and mean discrepancy, and SPICE refinement discrepancy
    electronics_spice_relative_tolerance = 1e-8 # dimensionless; ngspice solver relative tolerance for the RL reference circuit
    electronics_spice_absolute_tolerance = 1e-10 # A; ngspice solver absolute current tolerance for the RL reference circuit
    electronics_spice_case_timeout = 600.       # s wall-time budget per base vendor-model case; scaled by replay duration/refinement, preserving logs on timeout; not a numerical tolerance
    electronics_readout_spice_method = 'gear' # INA181 diagnostic integration method; verify time-step refinement with the unchanged vendor model
    electronics_readout_spice_order = 2        # maximum integration order for the INA181 diagnostic
    electronics_readout_spice_solver = 'klu'  # matrix solver for the ill-conditioned vendor macro-model; no change to its electrical equations
    electronics_readout_spice_compatibility = 'ps' # PSpice compatibility for the TI model, kept separate from the ADI LTspice diagnostic
    electronics_readout_spice_relative_tolerance = 1e-6 # dimensionless; vendor macro-model DC solver tolerance, with unchanged output discrepancy acceptance
    electronics_low_side_spice_steps_per_cycle = 2048 # maximum step denominator for INA181 switching transients, verified against twice this resolution
    electronics_readout_supply_rise = 1e-6    # s; power-on ramp excitation for vendor-model startup verification, not a selected supply specification
    electronics_replay_compression_fraction = 0.5 # fraction of numerical current-discrepancy limit per shunt input for replay simplification; final output refinement remains mandatory
    electronics_readout_startup_cycles = 16    # carrier cycles for power-ramped INA181 replay; adjacent final means must independently meet the unchanged settling limit
    electronics_readout_test_load = 10000.    # ohm; TI INA181 reference-test output load, not a selected ADC interface
    electronics_channel_bus_capacitance = 4.7e-6 # F; preliminary local bridge decoupling; shared regenerative/bulk network remains required
    electronics_analog_bypass_capacitance = 100e-9 # F; local current-sense amplifier supply bypass
    electronics_current_amplifier_gain = 10.   # V/V; INA241A1 candidate, retaining the existing shunt resistance
    electronics_analog_supply = 5.             # V; provisional local current-sensing supply
    electronics_current_reference = 2.5        # V; separate REF5025 current-sense zero reference; loading and noise qualification remain open
    electronics_readout_trial_shunt = 0.01    # ohm; total resistance of the candidate equal return-shunt pair; each physical leg uses half, with the flight model unchanged
    electronics_ina241_edge_spacing = 3e-6    # s; datasheet minimum edge separation for enhanced PWM rejection, not a guaranteed settling-to-accuracy bound
    electronics_max49925_gain_test_excursion = 0.8 # V about the reference; output-referred input range used for the published gain-error specification
    electronics_drv8962_deadtime = 300e-9     # s; typical datasheet dead time, not a guaranteed worst-case timing bound
    electronics_drv8962_on_resistance = 0.053 # ohm per FET; typical at 25 C and 5 A, not the hot maximum
    electronics_drv8962_input_delay = 600e-9 # s; typical IN-to-output delay, common to both legs in the representative circuit; no guaranteed skew bound
    electronics_drv8962_enable_delay = 2e-6 # s; maximum EN-to-output delay from DRV8962 Rev B table 6.5, distinct from typical IN/PWM propagation
    electronics_drv8962_supply_currents = (6e-3, 9e-3, 8e-6) # A/package; typical/max awake no-load and max sleep with VCC=DVDD, TI Rev B
    electronics_drv8962_wake_time = 1.2e-3 # s; maximum wake-to-output availability, with rails already powered
    electronics_drv8962_sleep_time = 120e-6 # s; minimum nSLEEP low duration to enter sleep (DRV8962 table 6.5), distinct from the shorter fault-reset pulse
    electronics_drv8962_turn_on_time = 1.3e-3 # s; maximum motor-rail UVLO crossing to output availability
    electronics_sar_analog_currents = (16e-3, 4.5e-3, 20e-6) # A/device; maximum ADS8698 conversion, standby and power-down AVDD currents at 5 V
    electronics_sar_wake_times = (20e-6, 15e-3) # s; datasheet standby and internal-reference power-down recovery waits; distinguish reference stabilization from driver wake
    electronics_reference_quiescent_current = 1.2e-3 # A; maximum REF5025 supply current, excluding delivered reference load
    electronics_shared_bulk_candidate = ('EEUFR1H102', 1000e-6, 0.2, 50., 0.016, 3.32) # MPN, F, tolerance, V, ohm at 100 kHz/20 C, A RMS at 100 kHz/105 C; Panasonic FR candidate; impedance is not a qualified broadband ESR bound
    electronics_pwm_interleave_span = 0.5 # fraction of a carrier spread across counter preloads for the interleaved PWM candidate; firmware deployment and phase-dependent sensing remain unqualified
    electronics_shared_source_tolerance = 0.02 # fractional design envelope for a regulated motor source including static regulation; component/source selection remains open
    electronics_shared_feed_resistance = 0.05 # ohm per eight-coil branch; exploratory combined source and distribution resistance, not a routed-board measurement
    electronics_shared_source_current = 10. # A per eight-coil branch; provisional source-current allocation for the coupled supply study
    electronics_shared_bulk_counts = (0, 1, 2) # candidate capacitors per logical eight-coil group for the regeneration screen; zero retains only existing local bypass, not a production placement
    electronics_bridge_output_capacitances = (100e-12, 1e-9) # F per FET; exploratory constant-Coss sensitivity points, not a DRV8962 manufacturer bound
    electronics_bridge_diode = (0.8, 0.02)    # V knee, ohm slope; exploratory body-diode approximation, not manufacturer characterization
    electronics_bridge_off_resistance = 1e9  # ohm; numerical off-state path in the representative switch model, not specified driver leakage
    electronics_bridge_spice_solver = 'sparse' # matrix solver for the physical bridge network; standalone amplifier fixtures retain their separate solver
    electronics_bridge_spice_steps_per_cycle = 4096 # maximum step denominator for finite-conductance/body-diode bridge carrier means; independently doubled before acceptance
    electronics_bridge_charge_tolerance = 1e-18 # C; explicit capacitor-charge truncation tolerance for milliohm return paths; refine against the unchanged current discrepancy limit
    electronics_bridge_absolute_tolerance = 1e-8 # A; nonlinear solve tolerance for the bridge, tightened with refinement; distinct from unchanged output accuracy acceptance
    electronics_bridge_voltage_tolerance = 1e-9 # V; node-voltage solver tolerance for bridge-state integration; independently tightened in refinement checks
    electronics_bridge_periodic_evaluations = 80 # maximum shooting-map evaluations for a periodic circuit state; acceptance still uses physical state closure
    electronics_bridge_settling_constants = 12 # coil L/R time constants before retaining the switching readout window
    electronics_readout_sample_guard = 100e-9 # s; provisional sample clearance before the next PWM edge; ADC aperture and jitter not yet selected
    electronics_drv8962_shunt_drop_limit = 0.3 # V; recommended maximum external PGND sense-resistor drop
    electronics_ina181_bandwidth = 350e3       # Hz; typical A1 bandwidth, used only for a first-order settling screen, not a guaranteed pulse-response bound
    electronics_acquisition_filter_resistance = 200. # ohm; candidate series output isolation/filter resistor; bounded switched/vendor verification passed; component corners remain open
    electronics_acquisition_filter_capacitance = 200e-9 # F; physical output capacitor, implemented as two parallel equal capacitors in the schematic
    electronics_acquisition_filter_cycles = 48 # carrier cycles for the loaded-amplifier startup check; final adjacent means still must pass the unchanged limit
    electronics_acquisition_sar_input_resistance = 1e6 # ohm; typical ADS8698 resistive input; internal bias/impedance variation still require bounds
    electronics_acquisition_sar_input_bias = 2.5 # V; ADS8698 input-leakage equivalent bias for the selected unipolar range, independent of the external INA181 reference
    electronics_acquisition_bulk_capacitance = 22e-6 # F nominal; ADC rail and reference bypass, requiring at least 10 uF effective capacitance after bias/tolerance
    electronics_acquisition_local_capacitance = 1e-6 # F; local ADC rail/reference high-frequency bypass and REF5025 input/output/NR capacitors
    electronics_bias_reference_capacitor_esr = 1. # ohm; intentional series resistance in REF5025 output-capacitor branch, within TI's low-noise recommendation
    electronics_readout_shunt_accuracy = (0.01, 110e-6) # fractional initial tolerance and maximum component TCR /K for WSLP12065L000FEA, including terminations
    electronics_shunt_land_pattern = (1.78, 1.93, 1.55) # mm; WSLP1206 2..5.9 mOhm recommended pad length, pad width and inner gap, Vishay 30122
    electronics_shunt_body = (3.20, 1.60) # mm; nominal WSLP1206 body outline, not an assembly clearance envelope
    electronics_footprint_courtyard_margin = 0.25 # mm beyond solder-pad extents for the custom shunt courtyard; copper spreading and Kelvin routes require additional area
    electronics_ina181_accuracy = (150e-6, 1e-6, 0.01, 20e-6) # input offset V, drift V/K, gain fraction, gain drift /K; maximum INA181A1 values near ground common mode
    electronics_sar_accuracy = (0.002, 3e-6, 0.0005, 4e-6, 5.) # offset V, drift FSR/K, gain fraction, gain drift /K, endpoint INL LSB; ADS8698 selected range with internal reference
    electronics_bias_reference_accuracy = (0.0005, 3e-6) # initial fractional accuracy and maximum drift /K; REF5025IDR high grade, distinct from ADC internal reference
    electronics_calibration_gain_allocation = 0.001 # fractional residual-gain design allocation after two-point calibration; requires a realizable calibration reference and procedure
    electronics_acquisition_cutoffs = (8000., 16000., 32000.) # Hz per pole; ideal buffered RC candidates for recorded-waveform screening, not selected components
    electronics_acquisition_orders = (1, 2, 3) # identical RC poles per candidate; loading and component tolerances require circuit verification
    electronics_acquisition_samples_per_carrier = (1, 2, 4, 16) # equally spaced ADC samples per PWM period, compared across all sampled starting phases
    electronics_acquisition_adc_bits = (16, 18) # ideal full-supply SAR quantizers for acquisition screening; not ENOB or adopted ADC specifications
    electronics_acquisition_grid_refinement = 4 # uniform samples relative to the base vendor transient resolution; repeat with twice this density
    electronics_acquisition_shunt_mismatch = (0.001, 0.01) # opposing fractional shunt errors for sensitivity tests; not selected resistor tolerances
    electronics_acquisition_sample_skews = (5e-9, 25e-9, 100e-9) # s; last-sample timing offsets for sensitivity tests, not a guaranteed ADC jitter specification
    electronics_acquisition_leg_skew = 50e-9  # s; relative leg propagation delay tested in the ideal switching bridge; no vendor tolerance is asserted
    electronics_acquisition_adc = 'ADS131M08IPBSR' # eight-channel delta-sigma comparison candidate; not selected or schematic-qualified
    electronics_acquisition_adc_channels = 8  # simultaneous channels per candidate ADC, not a final board partition
    electronics_acquisition_adc_resolution = 24 # nominal output bits; use input noise separately
    electronics_acquisition_adc_reference = 1.2 # V; candidate internal reference, giving +/- Vref input range at gain one
    electronics_acquisition_adc_osr = 128     # modulator samples per output; minimum datasheet OSR, for the candidate sinc3 filter
    electronics_acquisition_adc_sinc_order = 3 # steady-state digital-filter order at the candidate OSR
    electronics_acquisition_adc_clock_divisor = 2 # external clock / modulator clock for ADS131M08
    electronics_acquisition_adc_supply = 3.3  # V; candidate ADC supply, requiring a revised amplifier bias or level translation
    electronics_acquisition_adc_noise = 21.31e-6 # V RMS; typical gain-one/OSR-128 datasheet result at nominal clock; not a guaranteed bound at the proposed clock
    electronics_acquisition_adc_nominal_clock = 8.192e6 # Hz; clock used for the cited ADC noise measurements
    electronics_acquisition_adc_startup_clocks = 856 # external-clock cycles; datasheet startup/resynchronization settling at OSR 128
    electronics_acquisition_adc_frame_words = 10 # status, eight channel results and CRC in the default full SPI frame
    electronics_acquisition_adc_max_sclk = 25e6 # Hz; candidate datasheet serial clock maximum, not yet a selected PCB timing budget
    electronics_acquisition_adc_price = (1000, 2.9058, 'https://www.lcsc.com/product-detail/C2862610.html') # quantity, USD/chip, source observed 2026-10-20; no production quote
    electronics_acquisition_sar = ('ADS8698IDBTR', 18, 8, 500e3, 5.12, 15e3) # MPN, bits, multiplexed channels, total samples/s, input span V, typical input bandwidth Hz; technical comparison, not selected
    electronics_acquisition_sar_frame_bits = 40 # clocks; byte-aligned transfer containing the ADS8698 command and 18-bit result; local processor not selected yet
    electronics_acquisition_sar_serial_clock = 16e6 # Hz; proposed local SPI clock below the datasheet 18 MHz ceiling
    electronics_acquisition_sar_frame_guard = 20e-9 # s; combined CS setup/end allowance for the local serial timing screen
    electronics_regulation_samples_per_carrier = 2 # fixed acquisition burst size for the current paired-sample regulator candidate
    electronics_acquisition_sar_max_sclk = 18e6 # Hz; ADS8698 datasheet serial-clock ceiling
    electronics_regulation_compute_time = 5e-6 # s; per-channel compiled acquisition/reference/PI/PWM allocation; preserves the carrier latch, requiring ARM worst-case execution-time verification
    electronics_logic_supply = 3.3          # V; selected local MCU/watchdog/logic rail, distinct from the analog ADC supply
    electronics_local_controller = 'RP2354B' # candidate 48-GPIO MCU with stacked flash, eight coil loops per shared ADC; firmware and board timing not yet qualified
    electronics_logic_gpio_nets = (         # net assigned to GPIO0..47; PWM pairs use slices 0..7, host uses SPI0 and Hall interface uses SPI1
        'PWM_0_P','PWM_0_N','PWM_1_P','PWM_1_N','PWM_2_P','PWM_2_N','PWM_3_P','PWM_3_N',
        'PWM_4_P','PWM_4_N','PWM_5_P','PWM_5_N','PWM_6_P','PWM_6_N','PWM_7_P','PWM_7_N',
        'ENABLE_REQUEST_0','ENABLE_REQUEST_1','ENABLE_REQUEST_2','ENABLE_REQUEST_3',
        'ENABLE_REQUEST_4','ENABLE_REQUEST_5','ENABLE_REQUEST_6','ENABLE_REQUEST_7',
        'ADC_SCLK','ADC_CS_N','ADC_SDI','ADC_SDO','ADC_RESET_N','ADC_ALARM','AWAKE','FAULT_N',
        'HOST_RX','HOST_CS_N','HOST_SCLK','HOST_TX','WATCHDOG_PULSE','SAFE_OK','ARM_REQUEST','HOST_SYNC',
        'HALL_RX','HALL_CS_N','HALL_SCLK','HALL_TX','TEMP_DRIVER','TEMP_COIL','V_MOTOR_MONITOR','POWER_FAULT_N')
    electronics_watchdog_part = 'LTC6993IS6-2#TRMPBF' # rising-edge retriggerable hardware missing-pulse detector, shared by eight independently driven coils
    electronics_watchdog_set_resistance = 51100. # ohm; RC0402FR-0751K1L, above the LTC6993 minimum even with tolerance and temperature
    electronics_watchdog_divider_top = 976000. # ohm; RC0402FR-07976KL, manufacturer DIVCODE=1 divider
    electronics_watchdog_divider_bottom = 102000. # ohm; RC0402FR-07102KL, manufacturer DIVCODE=1 divider
    electronics_protection_resistor_accuracy = (0.01, 100e-6) # initial fractional tolerance and /K TCR bound for the selected RC0402 timing/divider resistors; aging not included
    electronics_protection_temperature = (-40., 85.) # C; circuit timing screening interval limited by the selected LTC6993I grade
    electronics_protection_reference_temperature = 25. # C; initial component tolerance reference for the protection screen
    electronics_watchdog_trigger_duration = 100e-9 # s; behavioral commit-pulse width, above the LTC6993 typical recognition width; pad-level timing remains unqualified
    electronics_watchdog_pulse_error = 0.044 # fractional full-temperature LTC6993 error at NDIV=8, distinct from its 25 C headline specification
    electronics_protection_logic_delays = (4.9e-9, 5.9e-9, 4.7e-9) # s; maximum 3.3 V/50 pF delays for SN74LVC1G11, SN74LVC1G74 asynchronous clear and SN74LVC2G08 at -40..85 C; PCB loads must be bounded
    electronics_logic_regulator_capacitance = 4.7e-6 # F; RP2354 input/output and regulator analog bypass, following the recommended native switching-regulator layout
    electronics_logic_bypass_capacitance = 100e-9 # F; per-pin MCU core/IO/USB/ADC/QSPI bypass, independent of the current-amplifier supply filter
    electronics_logic_regulator_filter = 33. # ohm; series feed for RP2354 VREG_AVDD
    electronics_logic_inductance = 3.3e-6     # H; polarity-marked Abracon AOTA-B201610S3R3-101-T, orientation and exact layout must follow the manufacturer
    electronics_logic_crystal_frequency = 12e6 # Hz; manufacturer-recommended ABM8-272-T3 crystal for the MCU PLL
    electronics_logic_crystal_capacitance = 15e-12 # F per crystal leg; PCB parasitics and oscillator startup still require verification
    electronics_logic_crystal_resistance = 1e3 # ohm; series crystal drive resistance at 3.3 V IOVDD
    electronics_logic_clock = 128e6 # Hz; RP2350-family candidate system clock, allowing integer carrier periods and PIO serial clocks; firmware timing remains unqualified
    electronics_logic_clock_max = 150e6 # Hz; published RP2350 maximum system clock, no overclocking assumed
    electronics_deadtime_compensation_current = 0.1 # A; smooth requested-current sign transition for provisional dead-time feedforward; bounded switching checks do not qualify every low-current case
    electronics_regulation_integral_gain_ratio = 0.375 # candidate integral-gain scale for the revised output filter; the flight regulator default remains unchanged
    electronics_regulation_switching_duration = 0.001 # s; resolved-switching command transition and settling window, with automatic peak-current stop and numerical refinement
    electronics_regulation_lifecycle_cases = (
        # Name, recorded initial-current fraction, requested-current fraction, bridge enabled; fractions use the regulated channel rating, duration uses electronics_regulation_hold.
        ('zero_hold', 0., 0., True),
        ('low_positive', 0., 0.05, True),
        ('low_negative', 0., -0.05, True),
        ('enable_positive', 0., 1., True),
        ('reverse_positive', -1., 1., True),
        ('release_hold', 1., 0., True),
        ('disable_positive', 1., 0., False),
        ('disable_negative', -1., 0., False),
    )
    electronics_regulation_commands = (0., 1., -1., 0.) # fractions of channel rating for startup, positive hold, reversal and release
    electronics_group_command_fractions = (1., -1., .75, -.75, .5, -.5, .25, 0.) # independent eight-coil commands; includes a stationary neighbour channel
    electronics_regulation_hold = 0.002       # s per constant command in the startup/reversal/release check
    electronics_regulation_settling_window = 0.0005 # s; final portion of each command plateau used to check existing tracking headroom
    electronics_regulation_resistance_factors = (1., 1.5) # exploratory total-series-resistance sensitivity, not a qualified hot-part bound
    electronics_regulation_filter_factors = (0.8, 1., 1.2) # exploratory amplifier/filter bandwidth sensitivity, not a manufacturer tolerance claim
    electronics_acquisition_sar_filter_order = 2 # datasheet internal LPF order; equal real poles matching typical -3 dB bandwidth are a screening approximation, not a manufacturer transient model
    electronics_acquisition_sar_snr = (90.5, 88.) # dB; typical/minimum ADS8698 SNR for the selected 0..5.12 V range, internal reference, at the datasheet test amplitude
    electronics_acquisition_sar_test_dbfs = -0.5 # dBFS; sinusoidal amplitude used for the cited ADS8698 SNR specification
    electronics_acquisition_sar_price = (100, 13.1351, 'https://www.lcsc.com/product-image/C2926478.html?whichImg=sch') # quantity, USD/chip, inspected LCSC tier on 2026-10-20; production-volume price unavailable
    electronics_readout_specs = (
        # Name, gain V/V, typical input-noise V/sqrt(Hz), maximum quiescent A; datasheet values for candidate screening, not qualified circuits.
        ('INA241A1', electronics_current_amplifier_gain, 62e-9, 3.2e-3),
        ('AD8410A', 20., 38e-9, 10.8e-3),
        ('MAX49925', 10., 150e-9, 7e-3),
        ('TPA132A1', 20., 80e-9, 4e-3),
        ('INA181A1', 20., 40e-9, 300e-6),
    )
    electronics_readout_price_observations = (
        # Candidate, description, published quantity, USD/unit, source; observed 2026-10-20, not production quotations or adopted BOM prices.
        ('AD8410A', 'AD8410A family starting price', 1000, 1.68, 'https://www.analog.com/en/products/ad8410a.html'),
        ('AD8410A', 'AD8410AWBRMZ-RL DigiKey reel', 3000, 2.34335, 'https://www.digikey.com/en/products/detail/analog-devices-inc/AD8410AWBRMZ-RL/21333813'),
        ('TPA132A1', '3PEAK TPA132A1-TS1R-S LCSC C44859266', 1000, 0.8091, 'https://www.lcsc.com/product-detail/C44859266.html'),
        ('INA181A1', 'TI INA181A1IDBVR LCSC C2058943', 1000, 0.2538, 'https://www.lcsc.com/product-detail/C2058943.html'),
    )
    electronics_placement_board_count = 2      # physical PCBs beneath one coil layer in the exploratory integrated-driver placement; not an adopted mechanical stack
    electronics_integrated_driver = ('DRV8962DDVR', 'Package_SO:HTSSOP-44_6.1x14mm_P0.635mm_TopEP4.14x7.01mm', 2) # MPN, provisional DDV library footprint, independent full bridges per package; manufacturer land-pattern verification remains open
    electronics_driver_package_envelope = (8.3, 14.1, 1.2) # mm; provisional DDV0044D maximum lead-span, length and height from the footprint's TI package drawing; clearance envelope, not a detailed DRV8962 vendor model
    electronics_integrated_bus_bypass = 10e-9  # F; DRV8962 local VM ceramic bypass; separate from shared bus bulk capacitance
    electronics_charge_pump_reservoir = 1e-6   # F; DRV8962 VCP-to-VM reservoir from the manufacturer application circuit
    electronics_charge_pump_flying = 100e-9   # F; DRV8962 CPH-to-CPL capacitor; voltage rating must cover the motor supply
    electronics_driver_ldo_bypass = 1e-6      # F; DRV8962 DVDD bypass; internal logic rail, not the precision current reference
    electronics_logic_pull_resistance = 10000. # ohm; default-disable/sleep pulldowns and open-drain fault pullup in the repeated driver sheet
    electronics_sense_placement = ((0.25, 0.35), (0.75, 0.35)) # shunt and amplifier centers as fractions of a two-pitch-wide, one-pitch-high placement cell
    electronics_bypass_placement_x = (0.125, 0.375, 0.625, 0.875) # fractions of sense-cell width for amplifier and driver support capacitors
    electronics_bypass_placement_y = 0.82      # fraction of sense-cell height for the support-capacitor row
    current_regulator_rate = 20000              # Hz; sampled current-regulator update rate, independent of the PWM carrier
    driver_switching_time = 1e-07               # s
    driver_hot_resistance = 0.08                # ohm
    driver_mosfet_gate_charge = 3e-08           # C
    gate_drive_voltage = 10                     # V
    logic_gate_voltage = 5                      # V
    current_command_rate = 4000                 # Hz; driver setpoint update rate, distinct from pose feedback, current-loop bandwidth and PWM
    pose_feedback_rate = 1000                  # Hz; conservative candidate pose-estimation/feedback rate, to be tested rather than minimized
    pose_controller_frequency = 20             # Hz; natural frequency of the candidate six-axis critically damped pose controller
    pose_controller_damping = 1                 # ratio; candidate second-order pose-controller damping
    pose_observer_frequency_ratio = 0.5         # ratio; observer pole frequency / pose-controller frequency; smoother recorded-noise candidate, coupled settling validation pending
    pose_observer_ratio_candidates = (0.5, 1., 2.) # ratios; recorded-sensor screening candidates relative to the unchanged pose-controller frequency
    qualification_sensor_latency = 0.001        # s; assumed maximum additional pose-estimate age at the feedback tick
    qualification_position_error = 0.00016      # m; bounded vector magnitude of injected position error, provisional sensing requirement
    qualification_angle_error = 0.005           # rad; bounded vector magnitude of injected orientation error, provisional sensing requirement
    qualification_mass_fraction = 0.1           # ratio; symmetric mass uncertainty about the bottom-heavy geometric king estimate
    qualification_com_fraction = 0.2            # ratio; symmetric COM-height uncertainty about the geometric king estimate
    qualification_inertia_fraction = 0.25        # ratio; symmetric uncertainty in each principal moment of inertia
    qualification_inductance_factors = (0.5, 2) # ratio; screening bounds on unmeasured self-inductance; mutual inductance remains unmodeled
    qualification_recovery_duration = 0.25       # s; local hover disturbance-recovery observation window
    qualification_initial_error_fraction = 0.25 # ratio; initial translation/tilt disturbance relative to their existing position/tip budgets
    qualification_noise_bias_fraction = 0.5     # ratio; share of bounded sensor error assigned to constant bias, remainder to a sinusoid
    qualification_integration_substeps = 2      # count; averaged mechanical/electrical integration steps per current-regulator update
    qualification_field_rate = 4000            # Hz; numerical magnetic-force refresh rate, independent of physical feedback and command rates
    qualification_refinement_factor = 2         # ratio; independent time and force-mesh refinement for the worst tested trajectory
    qualification_parallel_workers = 3          # processes; independent dynamic cases run with separate simulator geometry state
    qualification_convergence_fraction = 0.05   # ratio; numerical tolerance relative to tracking budgets, reserved current, and observed peak electrical power
    qualification_current_reserve_fraction = 0.1 # ratio; minimum unused channel-current capacity in the tested demand envelope
    qualification_current_transient_allowance = 0.1 # A; extra allocation headroom for regulator transients, beyond sense offset and command quantization
    setpoint_filter_passive_price = 0.003       # USD per 15.8k + 10nF pair (catalog target; production RFQ needed)
    setpoint_filter_cutoff = 1000               # Hz
    max_setpoint_bits = 12                      # count
    max_current_command_error = 0.005           # A
    current_loop_bandwidth = 2000               # Hz; assumed inner current-regulation bandwidth, not a sampling rate
    min_current_loop_samples = 8                # ratio; current-regulator samples per current-loop bandwidth
    min_setpoint_serial_headroom = 1.2          # ratio (parallel logical-board serializer lanes are sized above the quantization minimum)
    current_sense_resistance = 0.02             # ohm
    current_feedback_bits = 16                 # bits; provisional bipolar current-feedback precision candidate; coupled refinement and component qualification pending
    current_feedback_gain_bound = 0.005        # fraction; provisional residual current-sense gain error after calibration
    current_feedback_noise = 0.001             # A RMS; provisional independent noise per channel and regulator-rate sample
    current_feedback_seed = 1730               # deterministic seed for virtual current-sense calibration and noise
    airborne_check_duration = 0.002            # s; short coupled-interface run after sensor acquisition, not hover qualification
    airborne_reference_gap_fraction = 0.5      # fraction of the verified lower-to-upper gap span; interior hover target and release height
    airborne_checkpoint_interval = 0.001       # s simulated; maximum progress between atomic coupled checkpoints
    airborne_video_min_duration = 6            # s playback; minimum length for inspecting short or failed airborne trials, with slowdown labeled
    airborne_refinement_duration = 0.05        # s; initial recovery interval compared independently at finer time and magnetic resolution
    airborne_refinement_screen_duration = 0.01 # s; several Hall updates must converge before the longer refinement batch is allowed
    airborne_hover_duration = 1                # s; extended nominal hover after the local numerical comparisons pass
    airborne_motion_translation = (0.0003, 0., 0.) # m; provisional first local-flight displacement in board axes
    airborne_motion_rotation = (0., 0., 0.01)  # rad; world-axis rotation vector for the first local-flight command
    airborne_motion_start = 0.01               # s; initial stationary command before the first local movement
    airborne_motion_duration = 0.04            # s; smooth rest-to-rest translation and rotation interval
    airborne_motion_settle_duration = 0.2       # s; stationary endpoint observation after the first local movement
    airborne_settling_window = 0.1             # s; final continuous observation window for the move-and-hold settling screen
    airborne_settling_speed = qualification_position_error / airborne_settling_window # m/s; residual speed that would consume one position-error budget over the observation window
    airborne_settling_angular_speed = qualification_angle_error / airborne_settling_window # rad/s; residual angular speed that would consume one angle-error budget over the observation window
    airborne_readiness_duration = airborne_motion_start + airborne_check_duration # s; integration check covering startup and the beginning of commanded movement
    adaptive_max_step = 1 / current_regulator_rate # s; largest attempted interval, also clipped to hardware events
    adaptive_min_step = adaptive_max_step / 256 # s; smallest attempted interval before an explicit accuracy failure
    adaptive_clock_tolerance = 1e-12           # s; event-time roundoff snapping only, far below the minimum physical step
    adaptive_max_retries = 8                   # count; rejected attempts allowed at one physical time
    adaptive_growth_threshold = 0.125          # normalized local error; allow doubling below this threshold
    adaptive_position_tolerance = 1e-8         # m; provisional local full-step/half-step position difference budget
    adaptive_angle_tolerance = 1e-7            # rad; provisional local orientation difference budget
    adaptive_velocity_tolerance = 1e-5         # m/s; provisional local linear-velocity difference budget
    adaptive_angular_velocity_tolerance = 1e-4  # rad/s; provisional local world angular-velocity difference budget
    adaptive_current_tolerance = 1e-4          # A; provisional local physical-current difference budget
    adaptive_field_tolerance = 1e-7            # T; provisional local analogue Hall endpoint/path/filter difference budget
    joint_relative_tolerance = 1e-10          # dimensionless RK45 local relative tolerance; absolute scales reuse the existing state/field budgets divided by sqrt(state count)
    joint_work_tolerance_fraction = 1e-4      # fraction of total airborne energy allowance used as the local integrated-work absolute scale
    joint_quaternion_norm_tolerance = 1e-7    # dimensionless maximum quaternion norm drift before accepted-state normalization fails
    adaptive_reference_step = 1 / current_regulator_rate / 8 # s; fixed 6.25 us reference with the same exact hardware event scheduling
    adaptive_benchmark_duration = airborne_check_duration # s; initial coupled benchmark; longer sensitivity checks follow only after this passes
    adaptive_sensitivity_duration = 0.013      # s; bounded qualification through the previously divergent 12.25 ms interval
    comparator_input_offset = 0.005             # V (LM393 raw, removed by idle zero-calibration)
    current_sense_offset_residual = 0.001       # V (post-calibration drift: comparator + level-shift divider tempco over the operating range)
    max_current_offset_fraction = 0.05          # ratio of driver limit
    current_shunt_price = 0.0418                # USD (Milliohm HoJLR2512-2W-20mR-1% 75ppm, LCSC C2924538 at 4k; full-reel RFQ pending)
    current_shunt_power_rating = 2              # W
    current_comparator_channels_per_ic = 2      # count
    current_comparator_price = 0.0198           # USD
    current_frontend_passives_per_channel = 4   # count (RC filter + midpoint level-shift pair; 1% + matched arrays suffice - idle zero-cal removes statics)
    current_frontend_passive_price = 0.0035     # USD (1% thick-film + matched-pair arrays where tracking matters)
    driver_gate_passive_price = 0.0012          # USD
    driver_decoupling_price = 0.0197            # USD
    shift_register_outputs = 8                  # count
    gate_driver_half_bridges = 3                # count
    gate_driver_price = 0.2254                  # USD
    power_mosfet_price = 0.075                  # USD per dual N-MOSFET package; historical RFQ allowance, sourcing unverified
    shift_register_clock_rating = 20000000      # bits/s (74HC datasheet worst-case f_max at 4.5V, not the typical figure)
    shift_register_power_capacitance = 4.2e-11  # F
    shift_register_price = 0.028                # USD
    driver_serial_clock = 40000000              # bits/s
    smt_assembly_cost_per_joint = 0.0017        # USD
    max_bus_current = 120                       # A per split rail; baseline busbar/cabling limit checked against modeled peak load
    bus_distribution_price = 36.96              # USD
    rail_clamp_price = 5.0                      # USD per rail/zone; active regenerative-energy clamp allowance, design and sourcing outstanding
    rail_bulk_capacitor_price = 4.0             # USD (zone-level electrolytic; sizing needs ripple/inductance spec) [TO BE SOURCED]
    driver_channels_per_bulk_capacitor = 29     # channels (keeps the former 4 capacitors / 116-channel local decoupling density)
    bulk_capacitor_price = 0.25                 # USD, 330-470uF polymer local decoupling [TO BE SOURCED]
    supercap_cell_capacitance = 350             # F (Maxwell BCAP0350 P270 S18 snap-in)
    supercap_cell_max_voltage = 2.7             # V
    supercap_cell_working_voltage = 2.4         # V (series-count derating for cell life)
    supercap_cell_esr = 0.0032                  # ohm
    supercap_eol_capacitance_fraction = 0.8     # ratio (vendor end-of-life definition: 80% of minimum capacitance)
    supercap_eol_esr_factor = 2.0               # ratio (vendor end-of-life definition: 200% of maximum ESR)
    supercap_cell_price = 7.65                  # USD (Maxwell BCAP0350-P270-S18, DigiKey 1k tier; allocation advised)
    supercap_cell_mass_kg = 0.065               # kg (distributor-listed)
    supercap_balancer_price_per_cell = 0.15     # USD (balancing network share) [TO BE SOURCED]
    supercap_oring_price = 4.0                  # USD per protected buffer fan-out path; device/FET count must be re-sized from the modelled rail current [TO BE SOURCED]
    supercap_management_price_per_rail = 20.0   # USD (series-stack monitor + precharge/charge path + fuse + disconnect per rail bank) [TO BE SOURCED]
    midpoint_balancer_price = 40.0              # USD, active split-rail midpoint balance/protection allowance [TO BE SOURCED]
    midpoint_balancer_mass_kg = 0.3             # kg, controller + magnetics + protection allowance
    midpoint_balancer_current_rating = 5.0      # A, handles calibrated residual rail imbalance rather than traction current
    bus_droop_fraction = 0.1                    # ratio (allowed rail sag during buffer-fed burst; half depletion, half ESR)
    radiator_aluminium_price_per_kg = 12.0      # USD (extruded integral-fin heatsink, RFQ budget)
    playing_surface_price = 12.0                # USD
    motor_pcb_price_per_cm2 = 0.01063           # USD area allowance; physical segmentation is deliberately not fixed at this stage
    usable_bus_voltage_fraction = 0.9           # ratio
    hall_pitch_candidates = (30.0, 25.0, 24.0, 20.0, 100 / 6, 16.0, 15.0, 100 / 7, 12.5, 12.0, 10.0) # mm, sparse->dense; edge rows/columns are cropped to the motor outline
    hall_observation_window_side = 4            # count
    hall_sensor_mux_channels = 16               # count
    hall_sensor_price = 0.34758                 # USD (TI DRV5055A4QDBZR catalog tier; production RFQ needed)
    hall_sensor_mux_price = 0.2527              # USD
    magnet_cost_per_kg = 384.0                  # USD/kg; historical magnet cost allowance, production sourcing outstanding
    hall_adc_sample_rate = 500000               # samples/s
    hall_adc_native_bits = 16                   # bits; provisional external-converter target from fixed-timing replay, pending coupled and component qualification
    hall_adc_precision_candidates = (12, 14, 16) # bits; native-conversion candidates screened with unchanged sampling and physical noise
    hall_samples_per_sensor = 16                # count; samples averaged per Hall scan channel, independent of native ADC precision
    hall_max_condition = 16384                  # dimensionless; retained static Hall observability condition limit, independent of ADC precision
    hall_package_standoff = 0.6                 # mm (SOT-23 sensing element below PCB underside)
    hall_power_settle_time = 0.0001             # s (gated sensor group power-up + RC settle before burst)
    airborne_readiness_pause = airborne_motion_start + hall_power_settle_time # s; request a restart checkpoint during moving Hall acquisition, at the next accepted advance
    hall_gate_switch_price = 0.0206             # USD (GOODWORK AO3401A catalog tier; production RFQ needed)
    thermal_sensor_area_mm2 = 5000              # mm2 of motor area per distributed temperature sensor
    thermal_sensor_price = 0.0091               # USD (10k 1% 0402 NTC, LCSC catalog class) [TO BE SOURCED]
    hall_supply_voltage = 5                     # V
    hall_supply_current = 0.01                  # A max (TI DRV5055A4 at 5V datasheet maximum; ~6mA typical)
    hall_sensitivity = 12.5                     # V/T (TI DRV5055A4 nominal at 5V)
    hall_output_noise = 1.84e-05                # T rms typical (130nT/sqrtHz * sqrt(20kHz)); not a guaranteed maximum
    hall_linear_range = 0.169                   # T (TI DRV5055A4 at 5V)
    hall_sensor_bandwidth = 20000               # Hz (DRV5055 small-signal bandwidth; bounds independent noise samples)
    coil_field_subtraction_error = 0.005        # ratio (residual after Hall-array self-calibration of coil map + channel offsets, idle-board recalibration)
    position_error_gap_fraction = 0.1           # ratio (docking/landing error budget as fraction of flight gap; applies at hover currents)
    flight_position_error_gap_fraction = 0.25   # ratio; in-transit position-error budget relative to flight gap; neighbour clearance still requires verification
    pcb_copper_plane_thickness = 0.07           # mm (two 1oz solid planes in the 4-layer motor/control PCB allowance)
    radiator_standoff_below_pcb = 1.3           # mm (gap-filler bond line; clears 1.1mm SOT-23 Hall bodies under the PCB)
    radiator_slot_pitch = 5                     # mm (crosshatch eddy-break kerfs, island size)
    radiator_slot_web_thickness = 0.5           # mm (solid web left at plate bottom under the kerfs)
    radiator_slotting_price = 200.0             # USD (gang-saw crosshatch kerf pass on extrusion base, mid RFQ budget)
    potting_epoxy_price = 45.0                  # USD (alumina-filled 2.5 W/mK, coil bed allowance, RFQ budget)
    gap_filler_pad_price = 336.43               # USD (Laird Tputty SF560 5.6 W/mK, 10-pail public price per board; RFQ + selective dispensing could cut 67-80%)
    gap_filler_density = 3.4                    # g/cc (Tputty SF560)
    windings_per_coil_body = 1                  # count
    pcb_thickness = 1.6                         # mm
    frame_enclosure_mass_kg = 1.0               # kg
    board_electronics_mass_kg = 0.3             # kg
    blocker_hops_per_gap_move = 4               # count (2 blockers aside and back per knight gap move)
    potting_density = 1.8                       # g/cc (filled epoxy)
    bus_distribution_mass_kg = 0.8              # kg (busbar + zone cabling)
    ac_input_price = 8.0                        # USD (IEC inlet + fuse + mains switch + internal AC harness) [TO BE SOURCED]
    emi_filter_price = 12.0                     # USD; mains EMI filter allowance, switching-frequency suitability requires validation [TO BE SOURCED]
    enclosure_price = 40.0                      # USD (frame + skirt enclosure, 1.0kg mass already budgeted) [TO BE SOURCED]
    piece_control_flops = 20000                 # flop/update
    setpoint_dma_words_flops = 4                # flop per setpoint word (pack + DMA descriptor, FPGA does the delta-sigma)
    board_setpoint_fpga_count = 1               # count (one logical-board command engine; physical placement/partition remains open)
    board_setpoint_fpga_price = 25.0            # USD allowance for >=55kbit state RAM + parallel serializers [TO BE SOURCED / SYNTHESIS REQUIRED]
    board_setpoint_fpga_power = 1.0             # W allowance [TO BE VERIFIED BY SYNTHESIS]
    board_setpoint_fpga_solder_joints = 144     # count allowance
    control_node_adc_engines = 2                # independent Hall scan engines per regional MCU
    min_hall_scan_headroom = 1.1                # ratio
    min_control_compute_headroom = 2.0          # ratio
    control_node_mcu_price = 3.13               # USD, STM32G431KBT6 class
    control_node_mcu_solder_joints = 32         # count
    node_mcu_throughput_mflops = 170            # Mflop/s
    control_node_mcu_power = 0.4                # W
    host_power = 8                              # W
    psu_sizing_margin = 1.1                     # ratio; supply-sizing reserve above the modeled sustained hot-load requirement
    psu_family = 'Mean Well UHP-2500-36 adjusted to 40V' # retained supply model and voltage adjustment; hardware baseline, not a new selection
    psu_rating = 2498.4                         # W, full-bus continuous rating
    psu_price = 589.0                           # USD, DigiKey single-unit public price; RFQ at 100 boards
    psu_mass_kg = 4.07                          # kg, manufacturer packing specification
    psu_url = 'https://www.digikey.com/en/products/detail/mean-well-usa-inc/UHP-2500-36/11494555' # historical supply listing used for the BOM; not a refreshed quotation
    sliding_friction_coefficient = 0.3          # ratio; provisional kinetic friction during contact reset, requires surface/piece measurements
    minimum_contact_normal_fraction = 0.1       # fraction of piece weight that must remain supported by the board during reset
    coil_winding_clearance = 0.02               # mm; total radial clearance reserved beyond insulated wire width in each winding wall
    material_reference_temperature = 20.0       # C; reference for the copper-resistivity and magnet-remanence temperature models
    thermal_trace_step = 0.05                   # s; maximum integration step for transient plate/local heating
    shunt_power_derating = 0.5                  # fraction of nominal shunt power rating allowed by the dissipation check
    max_control_delay_fraction = 0.1            # fraction of the unstable growth time allowed for the ideal current-slew screen
    field_grid_samples = 41                     # samples per axis for the reference-plane magnetic-field grid
    matrix_rank_tolerance = 1e-9                # relative threshold, scaled by the largest matrix entry, for actuator/library rank
    singular_value_tolerance = 1e-12            # relative SVD cutoff for condition estimates; also a norm cutoff for normalized mode-library columns
    hall_rank_tolerance = 1e-6                  # relative threshold, scaled by the largest Jacobian entry, for Hall observability rank
    allocator_tolerance = 1e-11                 # SLSQP objective/convergence tolerance for normalized minimum-loss allocation
    allocator_residual_tolerance = 1e-7         # normalized constraint tolerance; also the LP primal/dual feasibility tolerance
    allocator_limit_tolerance = 1e-8            # relative numerical tolerance on allocated channel-current limits
    allocator_max_iterations = 300              # maximum SLSQP iterations; failure raises instead of substituting another solution
    position_derivative_step = 2e-5             # m; centered finite-difference displacement for six-axis mechanical stiffness
    hall_position_derivative_step = 1e-5        # m; centered finite-difference displacement for the Hall field Jacobian
    angle_derivative_step = 1e-4                # rad; centered finite-difference rotation for stiffness and Hall Jacobians
    nominal_force_mesh = 16                     # magpylib-force polyline meshing setting for nominal actuation calculations
    pose_force_mesh = 12                        # magpylib-force polyline meshing setting for the flight pose sweep
    verification_force_mesh = 8                 # magpylib-force polyline meshing setting for selected-coil authority and contact checks
    force_field_batch_points = 1024             # wire quadrature points per direct magnetic-field batch; changes memory use, not mesh resolution
    force_field_batch_candidates = (1024, 4096, 16384) # points per batch screened for runtime with identical wire samples and summation order
    force_field_workers = 3                     # CPU threads per force evaluation; independent batches retain their original summation order
    native_spline_workers = 4                  # CPU threads for native spline queries; point batches retain their original order
    native_spline_blas_threads = 1             # BLAS threads during native runs; prevents competing numerical-library thread pools
    atlas_grid_spacings = (0.001, 0.0005, 0.00025, 0.0002) # m; coarse-to-fine candidates for the local magnetic-field atlas screen
    atlas_spline_orders = (3, 5)               # polynomial degrees; standard cubic and quintic interpolation candidates
    coil_cache_grid_spacing = 0.00025         # m; coil-field grid verified by the matched-drive movement comparison
    coil_cache_spline_order = 3                # degree; cubic coil-field interpolation selected by accuracy and runtime checks
    atlas_padding_cells = 12                  # cells; sampled guard region around the query domain to reduce spline boundary effects
    atlas_max_storage_bytes = 768 * 1024 ** 2  # bytes; cap on three simultaneous grid-sized arrays during atlas construction
    atlas_validation_points = 4096            # points; independent random probes plus explicit domain-face probes
    atlas_validation_seed = 1740              # seed; reproducible off-grid field and pose validation
    atlas_validation_poses = 8                 # poses; random combined six-axis configurations in addition to nominal and extreme cases
    atlas_field_tolerance = adaptive_field_tolerance # T; maximum sampled vector-field interpolation error, independently of time integration
    atlas_matrix_relative_tolerance = 1e-4     # ratio; force and COM-torque column errors relative to the largest direct column norm in each block
    atlas_gradient_relative_tolerance = 1e-3   # ratio; field-Jacobian error relative to the largest direct derivative norm
    atlas_minimum_speedup = 2.0                # ratio; required median actuator-matrix speedup before coupled feasibility trials
    current_demand_mesh_refinements = (1, 2)    # diagnostic multipliers for segment, radial and axial resolution at the worst flight pose
    active_window_phase_samples = 6             # equally spaced samples across the combined coil-lattice repeat; handoff planes are additionally sampled
    active_window_handoff_offsets = (0.001, 0.0001, 0.00001) # m; shrinking distances on either side of a coil-window handoff for continuity diagnosis
    active_window_transition_fraction = 0.5     # fraction of the closest handoff-plane spacing used as the full smooth-blend width; must be between zero and one
    coil_radial_filaments = 2                   # filament samples across each winding wall in the finite-coil approximation
    max_coil_axial_filaments = 4                # upper bound on filament layers through the winding height
    axial_filament_spacing = 0.001              # m; target axial spacing used to choose the number of filament layers
    coil_coupling_heights_mm = (0.05, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 10.0) # mm; simulated coil heights defining the coupling interpolation domain
    pose_yaws_deg = (0, 45, 90)                 # deg; yaw orientations sampled for flight and contact allocations
    pose_phase_fractions = (0.0, 0.25, 0.5)     # fractions of the relevant coil/sensor pitch sampled in each lateral direction
    pose_tilt_fractions = (1.0, 0.5)            # fractions of the rim-clearance-limited tilt used for authority verification
    maximum_rim_sine = 0.95                     # dimensionless upper bound on the rim-drop/radius ratio before arcsine
    horizontal_force_directions = 8             # equally spaced horizontal demand directions around a full circle
    neighbour_yaws_deg = tuple(range(0, 91, 15)) # deg; sampled neighbour orientations for the powered-off snap-force screen
    neighbour_force_mesh = (4, 4, 4)            # cuboid mesh subdivisions along x, y and z for mutual-magnet force/torque
    eddy_force_mesh = (3, 3, 3)                 # cuboid mesh subdivisions along x, y and z for conductive-sheet image-force estimates
    mode_counts = (6, 8, 10, 12, 16, 20, 23)    # candidate current-basis sizes for the optional local mode-compression diagnostic
    setpoint_snr_bits_coefficient = 6.02        # dB per bit in the assumed delta-sigma setpoint resolution estimate
    setpoint_snr_offset = 13.0                  # dB offset in the assumed delta-sigma setpoint oversampling estimate
    setpoint_snr_oversampling_coefficient = 15.0 # dB per doubling of oversampling in the assumed setpoint resolution estimate
    wire_price_per_kg = 18.74                   # USD/kg; historical rectangular magnet-wire cost allowance, supplier quote outstanding
    piece_plastic_price = 1.4                   # USD per piece; shell material, inserts and finish allowance
    host_price = 61.96                          # USD per host compute module; historical BOM allowance
    host_carrier_price = 25.0                   # USD per host carrier/mainboard; historical BOM allowance
    report_scientific_lower_bound = 1e-3        # absolute nonzero value below which reports use scientific notation
    report_scientific_upper_bound = 1e6         # absolute value at or above which reports use scientific notation
    report_precision = 4                        # decimal places in fixed notation, significant digits in scientific notation
    report_label_width = 42                     # characters; minimum report label-column width
    power_mosfet_solder_joints = 8              # SMT solder joints per dual-MOSFET package for assembly cost
    gate_driver_solder_joints = 20              # SMT solder joints per gate-driver IC for assembly cost
    shift_register_solder_joints = 16           # SMT solder joints per shift-register IC for assembly cost
    setpoint_filter_solder_joints = 4           # SMT solder joints per two-component setpoint filter for assembly cost
    comparator_solder_joints = 8                # SMT solder joints per current-comparator IC for assembly cost
    passive_solder_joints = 2                   # SMT solder joints per two-terminal passive component
    hall_sensor_solder_joints = 3               # SMT solder joints per Hall sensor for assembly cost
    hall_mux_solder_joints = 24                 # SMT solder joints per Hall multiplexer for assembly cost
    gate_switch_solder_joints = 3               # SMT solder joints per Hall-group power switch for assembly cost
    coil_solder_joints = 2                      # winding termination joints per installed coil body for assembly cost
    gate_passives_per_half_bridge = 2           # gate passive components budgeted per half-bridge
    driver_half_bridges_per_coil = 1            # half-bridge channels per coil in the midpoint-return drive topology
    coil_bus_voltage_fraction = 0.5             # fraction of full bus available to one coil before voltage-headroom derating
    minimum_axial_filaments = 1                 # lower bound on filament layers through the winding height
    force_gradient_step = 1e-5                  # m; spatial finite-difference step passed to magpylib-force getFT
    thermal_convergence_tolerance = 1e-8        # C; temperature-change tolerance for steady/periodic thermal iteration
    thermal_max_iterations = 100                # maximum steady/periodic thermal iterations; nonconvergence raises
    cad_geometry_tolerance = 0.001              # mm; CAD bounding-box, clearance and STEP round-trip comparison tolerance
    cad_exploded_layer_spacing = 5              # mm; additional separation per layer in the illustrative exploded view only
    cad_preview_size = (1600, 1000)             # pixels; exported FreeCAD preview width and height
    cad_preview_margin = 1.2                    # ratio; camera-height expansion after fitting the CAD preview
    cad_overlay_transparency = 85               # percent; translucent stack overlays in FreeCAD, with no physical meaning
    cad_detailed_coil_cell = (-3, -3)           # lattice indices; one foreground coil resolved into conductor, enamel and terminals
    cad_patch_cells_per_side = 9                # odd count of physical square coils per side in the representative CAD patch, not the active control window
    cad_king_profile = ((0, 0.5), (0.06, 0.5), (0.09, 0.43), (0.13, 0.45), (0.17, 0.36), (0.24, 0.3),
                        (0.58, 0.14), (0.64, 0.23), (0.68, 0.25), (0.74, 0.17), (0.82, 0.2), (0.85, 0.12)) # candidate outer profile: (fraction of king height, radius / base diameter); CAD hollows this shape
    cad_king_cross_width_fraction = 0.4         # fraction of base diameter; candidate crown cross width
    cad_king_cross_thickness_fraction = 0.12    # fraction of base diameter; candidate crown stem and bar thickness
    cad_king_crossbar_height_fraction = 0.55    # fraction of crown height; visual crossbar centre
    layout_screen_window_side = 45              # mm; common square local support window for the layout comparison
    channel_screen_window_sides = (layout_screen_window_side, 60) # mm; retained local support and a wider diagnostic window to distinguish selection limits from topology limits
    channel_screen_yaws_deg = tuple(range(0, 360, 45)) # deg; full-turn yaw samples in the broader independent-coil comparison
    layout_screen_phase_samples = 4             # samples per local lattice axis for the initial static comparison
    channel_pilot_phase_points = ((0., 0.), (0.125, 0.375), (0.25, 0.5), (0.375, 0.125),
                                 (0.5, 0.75), (0.625, 0.875), (0.75, 0.25), (0.875, 0.625)) # normalized x/y samples within a repeating electrical cell; pilot only, not spatial convergence
    layout_screen_rotations_deg = (0, 45)       # deg; single-layer coil-array rotations relative to the existing magnet axes
    layout_refinement_phase_samples = 8        # samples per lattice axis for shortlisted layouts, including between the coarse parquet phases
    layout_refinement_yaws_deg = (0, 22.5, 45, 67.5, 90) # deg; additional yaw checks for the layout shortlist
    layout_plot_dpi = 160                       # pixels per inch; layout comparison diagram export resolution
    layout_plot_size = (1600, 700)              # pixels; wide format for the same-scale geometry comparison
    prototype_wire_outer_diameter = 0.297       # mm; maximum Grade 2 overall diameter for 0.250 mm wire, Elektrisola IEC table
    prototype_wire_resistance_per_m = (0.3345, 0.3628) # ohm/m at 20 C; manufacturer's minimum/maximum for the selected round wire
    prototype_turn_spacing = 0.01               # mm; extra packing allowance beyond maximum insulated wire diameter
    prototype_lead_length = 25                  # mm per end; free lead allowance included in resistance, excluded from turn coupling
    prototype_lead_separation = 1.2             # mm; spacing between the two inner-window lead paths
    winding_lead_bend_radius = 0.8              # mm; provisional centreline bend radius for free leads, not a supplier-qualified limit
    winding_layer_transition_bulge = 0.2       # mm; local end-turn excursion into the inter-coil gap or inner window to separate layer crossovers
    winding_terminal_pad_size = (0.8, 0.4, 0.035) # mm; width, stripped-contact length and copper thickness of provisional PCB pads
    winding_path_sample_pitch = 0.05           # mm; maximum chord length for conductor clearance screening
    winding_magnetic_segment_length = 0.25    # mm; reference CAD chord length for Hall-field polylines and legacy force comparisons
    winding_force_quadrature_order = 16       # Gauss nodes per curved CAD edge for force/torque; magnetic refinement multiplies the order
    hall_conductor_refinement = 8             # factor relative to reference chord length for the precomputed coil-to-Hall map; independently checked against curved CAD wire
    motion_time_step = 1 / 32000              # s; numerical motion step with direct magnetics per step, independently refined from hardware rates
    motion_contact_time_constant = 0.0005      # s; provisional stiff-board contact response, not measured material compliance
    motion_contact_damping_ratio = 1           # dimensionless; provisional critically damped contact, not airborne stabilizing drag
    motion_contact_impedance = (0.9, 0.95, 0.001, 0.5, 2) # MuJoCo solimp: lower/upper impedance, width (m), midpoint and power; numerical contact assumptions
    motion_solver_iterations = 100             # iterations; MuJoCo contact solver cap for interface verification
    motion_solver_tolerance = 1e-10            # MuJoCo solver stopping tolerance; convergence is checked independently
    coupling_excitation_fraction = 0.5         # fraction of current rating and initial resistive drop used for reference current/back-EMF stimuli
    coupling_flux_orders = (8, 16, 32)         # Gauss orders for independent magnetic flux and curved-wire field references
    coupling_refinements = (1, 2, 4, 8, 16)           # factors for independent coupling mesh, derivative and time refinement
    coupling_relative_tolerance = 1e-4         # dimensionless independent-reference error limit for reduced coupling verification
    adaptive_energy_relative_tolerance = coupling_relative_tolerance # accumulated absolute mechanical-work defect / gravity energy across the permitted flight-gap span
    coupling_order_bounds = (0.75, 1.25)       # observed first-order range for the smooth held-back-EMF benchmark
    coupling_variable_duration = airborne_refinement_screen_duration # s; bounded independent variable-coupling integration benchmark
    coupling_reference_tolerance = 1e-10      # dimensionless local tolerance on scaled independent ODE reference states
    motion_verification_tolerance = 1e-9       # absolute SI numerical comparison tolerance for known-force interface checks, not a physical accuracy claim
    motion_check_duration = 0.01               # s; short airborne known-force/torque verification, not a levitation acceptance duration
    motion_check_orientation_deg = (3, -2, 17) # deg; mixed-axis pose for verifying world/body force and inertia transforms
    motion_magnetic_check_steps = 4            # direct force evaluations in the prescribed-current interface check; not a closed-loop trial
    motion_contact_penetration_limit = 0.00005  # m; numerical contact-check allowance, not an adopted manufacturing clearance
    motion_mesh_angular_tolerance = 0.1         # rad; CAD triangulation angular tolerance for MuJoCo display/collision meshes
    motion_preview_background = (0.92, 0.94, 0.97) # RGB fractions; light background for the MuJoCo assembly preview
    motion_video_frame_rate = 30                # frames/s; encoded diagnostic video playback rate, independent of the physics step
    motion_video_slowdown = 20                  # ratio; playback seconds per simulated second to expose contact motion
    motion_video_font_size = 24                 # pixels; diagnostic labels in the rendered motion video
    motion_video_side_azimuths = (0, 90)         # deg; perpendicular side cameras for inspecting the piece-to-board gap
    motion_video_gap_distance_radii = 3         # base radii; side-camera distance for a close view of the gap
    motion_video_surface_opacity = 0.35         # alpha; playing-surface visibility in the video gap views
    hall_virtual_seed = 1729                    # deterministic seed for reproducible virtual Hall calibration errors and noise
    hall_validation_noise_seeds = (1731, 1732)   # seeds; independent post-release Hall noise for observer screening, with calibration and recorded current readings held fixed
    hall_calibrated_offset_bound = 0.00005      # T; provisional residual offset bound after calibration, not a guaranteed sensor specification
    hall_calibrated_gain_bound = 0.005          # fraction; provisional residual Hall sensitivity error after calibration
    hall_pose_tilt_bound_deg = 10               # deg; roll/pitch search bounds for local upright-piece initialization
    hall_initial_yaws_deg = (-180, -135, -90, -45, 0, 45, 90, 135) # deg; fixed initial yaw search grid, never derived from true pose
    hall_initial_xy_fractions = (-0.5, 0, 0.5) # half-window fractions; fixed lateral initialization grid within the selected sensor patch
    hall_initial_fit_count = 8                  # candidates; lowest-residual coarse seeds refined for local initialization
    hall_fit_max_evaluations = 200              # evaluations per local least-squares solve; failed convergence is rejected
    hall_fit_tolerance = 1e-8                   # dimensionless; scaled least-squares stopping tolerances
    hall_fit_residual_limit = 5                 # modeled uncertainty multiples; maximum RMS field residual for accepting a static pose
    hall_fit_ambiguity_margin = 1               # modeled uncertainty multiples; rival fit RMS margin considered indistinguishable
    hall_tracking_duration = 0.1               # s; local moving-scan diagnostic duration, not a full-board travel trial
    hall_tracking_velocity = (0.02, -0.01, 0, 0, 0, 0.5) # m/s then rad/s; imposed initial linear/body angular velocity for sensor-only motion checks
    hall_tracking_acquisition_duration = 0.025  # s; observer acquisition interval before estimates are marked usable
    hall_tracking_max_scan_gap = 3 / pose_feedback_rate # s; maximum delivery or measurement gap before tracking is invalidated
    hall_tracking_dropout_interval = (0.04, 0.045) # s; deliberately omitted scan-start interval for the loss/reacquisition diagnostic
    hall_field_batch_frames = 64                # poses per vectorized magnetic field evaluation; limits working memory without changing physics
    king_magnet_pocket_clearance = 0.1          # mm per side; adhesive space outside the unchanged contiguous Halbach array
    king_magnet_bond_thickness = 0.1            # mm; adhesive layer above magnets under the shell's structural roof
    king_bond_density = 0.0012                  # g/mm3; provisional unfilled adhesive density for CAD mass accounting
    prototype_corner_segments = 4              # segments per quarter-circle in the rounded-coil magnetic approximation
    prototype_handoff_taper = 5.0              # mm; distance over which a winding's allowed current smoothly reaches zero at the window boundary
    prototype_path_samples = 33                # samples over one lattice repeat; the most demanding path is rerun at twice the spatial resolution
    prototype_path_angles_deg = (0, 45, 90)     # deg in lattice coordinates; straight motion paths across local coil transitions
    prototype_path_offsets = (0.0, 0.5)         # fractions of the short lattice pitch transverse to each handoff path
    prototype_path_yaws_deg = (0, 45, 90)        # deg; held piece orientations for the prescribed-motion handoff screen


class Constants:
    ltc6993_time_per_resistance = 1e-6/50000. # s/ohm; datasheet oscillator transfer, not a chosen timeout
    ltc6993_divcode = 1                     # hardware divider code selected by 976k/102k; active-high output, divide by eight
    ltc6993_divisor = 8                     # timer divider corresponding to DIVCODE=1
    ltc6993_divcode_levels = 16             # number of internal DIV ADC codes
    ltc6993_divcode_margin = 0.015          # allowable absolute deviation of DIV/V+ from code center
    ltc6993_set_resistance_range = (50000., 800000.) # ohm; specified timing resistor range
    ltc6993_divider_impedance_limit = 500000. # ohm; maximum DIV Thevenin impedance
    ltc6993_divider_leakage = 10e-9         # A; maximum DIV input current
    ltc6993_startup_master_cycles = 500    # typical startup in oscillator periods, not a guaranteed timeout

    # Board definition and physical/material reference values; units are explicit for each entry.
    rp2350_pwm_compare_bits = 16             # width of each A/B compare field in the PWM CC register
    rp2350_pio_word_bits = 32                  # width of PIO shift registers and FIFO words
    rp2350_pio_immediate_bits = 5              # SET instruction immediate width
    rp2350_pio_delay_side_bits = 5             # instruction bits shared between side-set and delay
    ads8698_serial_timing = (30e-9, 10e-9, 30e-9, 5e-9, 5e-9, 825e-9, 1175e-9) # s; minimum CS-to-falling-clock, last-clock-to-CS, CS high, SDI setup/hold, maximum conversion and minimum acquisition
    ads8698_command_bits = 16                   # SPI command word width; input latched on falling SCLK edges
    ads8698_register_bits = 8                   # program register data width
    ads8698_address_shift = 9                   # register address starts at command bit 9
    ads8698_write_bit = 8                       # write-enable command bit
    ads8698_manual_command = 0xC000             # manual channel-zero selection opcode
    ads8698_manual_channel_shift = 10          # manual channel selection field offset
    ads8698_feature_register = 0x03             # SDO format and alarm configuration register
    ads8698_first_range_register = 0x05         # channel-zero input-range register; subsequent channels are contiguous
    ads8698_unipolar_range_code = 0x06         # input range 0..1.25*Vref, or 0..5.12 V with the internal reference
    ads8698_channel_tag_format = 0x01           # conversion result followed by the four-bit channel address
    ads8698_channel_tag_bits = 4                # width of the optional returned channel address
    board_squares_per_side = 8                  # chess squares along each side of the playing area
    ndfeb_remanence_br = 1.36                   # T; baseline N48SH remanence at material_reference_temperature; supplier grade/lot verification outstanding
    ndfeb_br_tempco = -0.0011                   # 1/K; assumed reversible NdFeB remanence coefficient relative to material_reference_temperature
    ndfeb_density = 0.0075                      # g/mm3; NdFeB mass density
    plastic_density = 0.0012                    # g/mm3; piece-shell material density
    copper_resistivity = 1.724e-08              # ohm.m; copper resistivity at material_reference_temperature
    copper_resistivity_tempco = 0.00393         # 1/K; copper resistivity coefficient relative to material_reference_temperature
    aluminium_resistivity = 2.82e-08            # ohm.m; baseline aluminium resistivity for the conductive-sheet drag model
    copper_density = 8960                       # kg/m3; copper mass density
    gravity = 9.80665                           # m/s2; gravitational acceleration
    vacuum_permeability = 1.25663706e-06        # H/m; magnetic permeability of vacuum
    fr4_density = 0.00185                       # g/mm3; PCB laminate mass density


if __name__ == "__main__":
    from methodology.design import main

    main()
