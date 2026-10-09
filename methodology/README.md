**Levitating chess engineering model**

The model distinguishes single-piece levitated movement from a simultaneous contact reset. Product behaviour is described in `PRODUCT_VISION.md`; quantitative requirements, assumptions, search candidates and numerical settings are owned by `Inputs`, `Fixed` and `Constants` in `model.py`.

**Repository layout**

The loose files at the repository root match the last committed layout. Supporting work lives here:

```text
model.py                         Inputs / Fixed / Constants; report entry point
levitation_sim.py                shared magnetic and allocation physics
last_run.txt                     latest main engineering report
methodology/
  design.py                      geometry, winding search, budgets and report calculations
  current_demand.py               static force/current diagnosis
  qualification.py               delayed closed-loop simulation and convergence checks
  tests/test_model.py             regression checks
  requirements.txt               pinned calculation dependencies
  cad/                           build123d geometry, FreeCAD documents and CAD checks
  results/                       saved JSON evidence and qualification plot
  README.md                      usage, boundaries and cleanup record
  CURRENT_DEMAND.md               current-demand findings
  QUALIFICATION.md                prototype targets, evidence and remaining limits
  MODEL_AUDIT.md                  historical audit
  COIL_LAYOUT_HISTORY.md          evidence and uncertainty behind the stacked layout
  MODELING_APPROACH.md            modeling scope, tools and shared-data approach
  NEXT_STEPS.md                   coil, thermal and electronics development sequence
```

Calculation code imports the three parameter classes; it does not execute the main script. `levitation_sim.py` remains the single shared physics implementation and receives settings through `SimGeometry`. There are no duplicate compatibility modules or copied parameter tables. Numerical/search settings remain in the same three classes to respect the project's parameter-ownership rule; they are distinguished from adopted hardware targets by their comments.

The adopted conductor thickness and axial layer count are now explicit `Inputs` fields. Wire width, turn count, insulated height and resistance still derive from the same geometry and material settings. Normal reports and dynamic qualification use this adopted winding. The optional winding search compares alternatives without changing the adopted targets.

**Run and test**

Run these commands from the repository root, using a Python environment compatible with the pinned packages:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r methodology/requirements.txt
.venv/bin/python -m unittest discover -s methodology/tests -v
.venv/bin/python model.py
```

`model.py` writes `last_run.txt` only after the calculation completes. Importing `model` does not execute the simulation. `methodology.design.calculate_model()` returns the calculated report sections, BOM and acceptance checks. `python model.py --winding-search` includes the optional winding comparison; the returned `sweep` is otherwise `None`. `python model.py --mode-analysis` includes current-mode analysis. `--output PATH` saves a separate main report. `python -m methodology.design` provides the same report entry point. `methodology.design.calculate_model(include_mode_analysis=True)` also evaluates the optional current-mode diagnostic; it does not reduce the driver count or BOM. Optimizer errors and invalid numerical inputs raise exceptions rather than selecting an alternate result.

The pre-existing `.venv` in the audited workspace was created with Python 3.12 but its interpreter symlink currently resolves to Python 3.14. Its compiled packages therefore cannot be used with that interpreter. Validation used an isolated temporary installation of the pinned dependencies; the existing environment was not overwritten.

**Interpretation**

This is a migrated engineering model, not an accepted hardware design. `StatusChecks.accepted` requires both passing numerical checks and an empty list of unverified requirements. A completed calculation can therefore report `NOT ACCEPTED`. The implementation retains the current bus, driver, cooling and supply as hardware baselines; the winding screen is not a complete hardware optimization.

- The flight count is separate from reset participation. Changing flight concurrency does not reduce the reset to one piece.
- Contact motion includes static and sliding friction, optimized vertical magnetic force with a positive normal-reaction constraint, and support-moment limits inside the base footprint. Flight and contact allocations include an approximate conductive-sheet drag force.
- Peak-current allocation calculates the minimum channel rating over sampled targets. Minimum-loss allocation then uses that rating as a common constraint for the mode. These are demand calculations: a required rating exceeding installed hardware produces a failed check, not an allocation silently clipped to the driver limit.
- Contact-reset heat/power is currently an isolated-piece sum. A two-touching-piece, opposing-motion probe additionally solves one shared-coil system, including mutual magnetic force and torque. This probe is not a proof for all 32 pieces or arbitrary placements. Global reset routing, full-board boundary coverage and stationary-neighbour disturbance remain unverified.
- Thermal calculations use separate reset, grind and hammer/cascade workloads. Per-cell transit heat uses a stationary-footprint bound. Sustained hot equilibrium and periodic local hammer heating are checked within the magnet material range. `inf` for a thermal result means there is no admissible modeled temperature within that range, not a predicted physical infinite temperature. The failure reason is printed. Thermal screening is not experimental validation.
- The power model sizes deficits against both normal flight and back-to-back resets. Zero required buffer capacity produces no supercapacitor management/protection costs. The baseline PSU is not automatically downsized.
- Control screening retains the nominal six-axis instability-margin estimate as a diagnostic. The candidate pose rate is now an explicit `Fixed` setting used by both the hardware report and the dynamic simulation; it is not a demonstrated minimum.
- The Hall-pitch search fails explicitly if no candidate passes. Whole-board scan sizing remains a baseline architecture; local high-rate scanning is a future optimization.
- Parked-hover and showpiece requirements are retained. The sustained fan-assisted workload has been reduced to 10 composite moves/min at the user's request; the passive workload remains one move per 6 s. T2 refers to the model-owned settings. Individual flight timings are unchanged, so this reduces average load without reducing the peak current or command-rate demand of a flight.

**Current-demand diagnosis**

`python -m methodology.current_demand` reproduces the force/torque demand breakdown using the adopted winding and existing operating allocator, and writes `methodology/results/current_demand.json`. It includes same-pose constraint comparisons, COM sensitivity and numerical mesh refinement. It also evaluates a nearest-window candidate across a complete combined coil-lattice period, checks physical-channel current changes at window handoffs, and probes the motor-area corners and edge midpoints. The default report retains fixed-window allocation until a continuous commutation policy and neighbour constraints have been established. See `CURRENT_DEMAND.md` for findings and remaining steps.

The king is the reference piece. Its existing shell-mass approximation is retained: a uniformly scaled hollow bounding cylinder plus the bottom magnet array. COM and both rotational inertias now follow from those same component masses and dimensions, using the parallel-axis theorem. This is the requested simple approximation, not a CAD-derived mass distribution. The old independent COM-height fraction has been removed. The preliminary winding screen now includes all configured horizontal force directions, but still uses reference-height coupling and omits conductive drag; selected-geometry allocation remains necessary.

Every local coil has an orientation and integer lattice indices identifying its physical channel. Changing the window centre or size preserves the physical lattice. Edge probes shift the complete window inward to keep every winding inside the motor footprint. Inactive coils have zero current when comparing successive windows. The handoff diagnostic compares abrupt selection with a smooth weighted blend of overlapping, independently balanced current patterns. It checks force/torque balance and channel limits after blending; it does not establish delayed closed-loop tracking or stationary-neighbour compatibility.

**Candidate dynamic qualification**

`python -m methodology.qualification` recalculates magnetic measurements for the adopted winding and writes `methodology/results/qualification.json`. Use `--case NAME` to run a selected scenario and `--output PATH` to keep a separate report. See `QUALIFICATION.md` for the tested scope, results and hardware implications. The corrected controller passes all nine local scenarios. The current-reserve-critical accelerating case converges under independent integration-step, magnetic-field-refresh and force-mesh refinement. These are conditional prototype results; complete-product qualification remains false. `results/qualification_previous_refinement.json` and `results/qualification_numerics.json` retain the earlier failure and its diagnosis.

The runner reuses the existing finite-magnet force calculation, winding selection and current allocators. It adds nonlinear rigid-body motion, delayed pose feedback with a position/velocity/disturbance observer, bounded sensor errors, quantized and filtered current commands, a sampled PI current regulator, voltage saturation and independent RL coil dynamics with motional back-EMF. Smooth window blending is shared with the current-demand diagnostic. Physical outgoing currents decay through ideal rail clamps before their channels become open circuit. Future inactive coils do not acquire artificial regulator-offset currents.

The cases test local hover recovery and fast one-period flight segments, including handoffs and board edges, with provisional mass/COM/inertia and thermal/electrical uncertainty. The case with the least current reserve is rerun independently with finer integration steps, field refresh and force mesh. Convergence checks include common-time position traces, current, power and acceptance outcomes. A sampled pass is conditional on those assumptions and does not qualify full lift/fly/land trajectories, large tilt/yaw manoeuvres, neighbours, the complete electronics implementation or thermal safety. No failure is converted into a passing result by relaxing the product limits. This runner does not search for minimum frequency or minimum power.

The averaged-motion candidate uses 5.5 A/channel, a 40 V full split bus, 1 kHz pose feedback, 4 kHz current commands and a 20 kHz current-regulator update rate (2 kHz bandwidth). The separate split-rail RL ripple screen sets a provisional 160 kHz PWM carrier and 6.5 A instantaneous power-stage target. The former 20 kHz carrier had excessive ripple under the assumed low-inductance bound. The main report includes conservative ripple and switching losses and retains its power/thermal failures. The carrier screen is not a switched electromechanical simulation; real winding inductance, sensing, driver transients and topology require validation.

**Parameter ownership**

`Inputs` contains requirements and design/search choices. `Fixed` contains implementation, material, numerical and costing assumptions. `Constants` contains physical/material constants and board definitions. The simulator receives those settings through `SimGeometry`; it does not import `model.py` or maintain a second tuning table. Algebraic factors, SI conversions, vector dimensions, coordinate bases and array indices are mathematical structure rather than tunable settings.

Board, piece, wire and stack dimensions in the parameter classes use millimetres; `max_tip_position_error` uses metres. Motion times use seconds, currents amperes, voltages volts, frequencies hertz and temperatures degrees Celsius. `SimGeometry` converts the geometry used by the magnetic library to SI. `Piece` is the authoritative source for mass, COM height and the approximate inertias; the simulator consumes those quantities.

New provisional assumptions for contact reset are explicitly stored in the input classes: reset duration, breakaway duration, sliding friction and minimum contact normal force. They are modeling assumptions for comparison, not measured material properties or newly approved product requirements. Resting friction remains independently specified for breakaway/no-snap calculations. Wire dimensions include insulation and winding clearance, but the candidate wire/build process is still not supplier-confirmed. The BOM is a cost allowance for the configured production volume, not a fresh vendor quotation.

**Verification coverage**

`tests/test_model.py` contains 46 regression tests covering delayed disturbance estimation, regulator/carrier clock independence, periodic RL ripple bounds, trajectory/current refinement checks, allocator wrench/current constraints, the difference between individual and shared-coil feasibility, insulated winding fit, zero-capacity buffer costs, interpolation-domain rejection, reset-population isolation, timing rejection, neighbour force/torque shape, thermal-domain failure reporting, optional reduced-current-basis calculations, and parameter/default-setting regressions. A full run exercises the finite-magnet/coil calculations, selected-geometry authority, contact/flight allocations, sensing, thermal accounting and reporting. Tests passing does not make the engineering acceptance checks pass.

`MODEL_AUDIT.md` is the historical pre-migration audit. Its source fingerprints and line references describe that earlier snapshot.

**Cleanup and saved evidence**

This reorganization preserves the physics implementation, existing numerical settings, parameter comments and saved simulation results. The root model's calculation definitions moved to `design.py`; callers use those definitions directly. Importing `model` now needs only the Python standard library and exposes no calculation API. Code that previously imported calculation classes from `model` must import them from `methodology.design`.

Saved JSON and the plot in `results/` were moved without changing their bytes. Their embedded source fingerprints describe the executions before this reorganization, not the current paths or current source hashes. No old fingerprint was relabeled as a new run. New diagnostic runs fingerprint `model.py`, `levitation_sim.py`, `design.py`, the runner and the shared fingerprint helper using repository-relative paths. `MODEL_AUDIT.md` remains a historical document: its original line references are not current navigation targets.

The cleanup was checked with the regression suite, command-line entry points, source-definition comparisons, saved-artifact checksums and a fresh main engineering report comparison. Every engineering value and acceptance result matches the preceding report exactly; only the now-optional winding-comparison section is omitted. `last_run.txt` contains this fresh report. The full dynamic campaign was not repeated for this structural change. Passing software checks does not resolve the existing thermal or full-product qualification failures.

See [MODELING_APPROACH.md](MODELING_APPROACH.md) for the agreed modeling direction and proposed tools, and [NEXT_STEPS.md](NEXT_STEPS.md) for the revised coil, thermal and electronics sequence.

The first geometry reference is available in [cad/README.md](cad/README.md), including assembled and exploded FreeCAD documents and a detailed winding view. CAD and the electrical calculation share `WindingGeometry`; the CAD environment can also run the existing regression suite. This geometry does not yet resolve winding manufacture, lead routing or thermal performance.
