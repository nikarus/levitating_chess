# Single-piece flight checklist

This is the engineering checklist for qualifying one actively controlled piece. It supplements the user-owned [product tests](../../PRODUCT_TESTING.md); it does not change them. The [simulator reference](SIMULATOR.md) owns numerical verification, [MODELING_APPROACH.md](MODELING_APPROACH.md#current-airborne-interface) owns measured results, and [NEXT_STEPS.md](../NEXT_STEPS.md) owns execution order. Keep those roles separate.

Each entry is a family of parameterized cases, not one animation or a promise to test every possible trajectory. Define amplitudes, heights, angles, rates, accelerations, durations, uncertainty ranges and acceptance thresholds in `Inputs`, `Fixed` or `Constants` in [model.py](../../model.py), with descriptive comments. Missing criteria must be defined before a case can qualify. The same controller must follow the different references without animation-specific corrections or access to physical truth.

## Current coverage

Short hover and one small combined sideways/yaw trial pass their recorded limits. These are partial evidence, not completion of the families below. The [movement evidence](../results/local_motion/trial/verification.json) and [tracking plot](../results/local_motion/trial/tracking.png) retain the current baseline. Endpoint settling, sustained control, broad motion, contact and other piece geometries remain unqualified.

## Basic airborne control

- [ ] **SP01 — Acquisition, hold and settling.** Acquire pose from sensors; hold at representative positions and heights; settle after movement; measure residual velocity, oscillation and drift over the required observation period. Stationary pre-acquisition does not count as controlled takeoff.
- [ ] **SP02 — Translation and braking.** Move in both board directions and diagonals; accelerate, cruise, brake, stop and reverse. Include different positions relative to the coil lattice and the intended speed/acceleration limits.
- [ ] **SP03 — Vertical movement.** Raise, lower, stop and reverse within the supported gap range, including low- and high-gap operation. A controlled vertical excursion is distinct from a ballistic jump.
- [ ] **SP04 — Tilt.** Tilt forward/backward, sideways and in intermediate directions; hold and return upright. Check both rotation tracking and the lowest rim/surface clearance, including orientations near the allowed boundary.
- [ ] **SP05 — Rotation.** Turn in both directions, including half and complete turns, braking and reversal. Verify continuous command/estimator behavior across orientation wrapping; an identical endpoint orientation does not prove a full turn was executed.

## Ordinary chess movement

- [ ] **SP06 — Coil and sensor transitions.** Travel across control-window boundaries, including braking or reversing near a transition. Retain winding-current decay and sensor timing. Include board/buffer edges only after their geometry and supported field domain exist. The current local controlled trial does not implement these handoffs.
- [ ] **SP07 — Takeoff.** Acquire while supported, build force, leave contact and reach the airborne target. Verify the contact mechanism first; test relevant initial alignment, tilt and surface variation. Starting a simulation already airborne does not satisfy this case.
- [ ] **SP08 — Landing.** Approach from different travel directions, reduce height, make contact and settle while currents are reduced. Include residual lateral motion, allowed tilt, friction variation and rebound. Preserve existing failed contact evidence until replacement verification passes.
- [ ] **SP09 — Complete move.** Execute acquisition, takeoff, travel, reorientation, braking, landing and release of active support as one continuous run. Check cumulative drift and electrical demand, not only separately passing phases.

## Combined and expressive movement

- [ ] **SP10 — Curved paths.** Follow circles, figure-eights and direction-changing paths, including smooth starts/stops and repeated cycles. Exercise changing acceleration rather than only constant-speed travel.
- [ ] **SP11 — Simultaneous translation, rotation and tilt.** Combine sideways/vertical movement with yaw and changing tilt; include rotation while tilted, half/full turns and changes of rotation direction. Screen joint force/torque demand: individually passing maxima need not be feasible together.
- [ ] **SP12 — Oscillation and coordinated dance.** Shake by repeatedly tilting and reversing; vary rate within the declared limits. Move the base while keeping a selected point near the top stationary, then combine that constraint with supported motion. Evaluate the selected-point tracking error as well as pose, clearance and current limits.

## Coverage applied across the families

- [ ] **Piece diversity.** Establish the king baseline, then introduce the knight's asymmetric mass distribution during basic-motion testing, before the expressive-motion campaign. Use each representative piece's CAD mass, centre of mass and full inertia tensor, with uncertainty. Select representative cases for other piece geometries by force, torque, clearance and sensing difficulty; do not assume the king bounds every case.
- [ ] **Disturbance recovery and construction variation.** Keep existing sensor/current noise, quantization and calibration errors enabled. Add bounded pushes, torque disturbances, sensing interference, noise realizations and plausible mass/magnet/coil/electrical variation as each family matures. Distinguish interference affecting measurements from external fields that also exert forces or induce currents. Verify recovery, not merely survival until the run ends.
- [ ] **Fault response.** Specify and test responses to missing/delayed readings, failed acquisition, current/voltage saturation and impending departure from the supported region. Distinguish controller recovery or a defined physical response from the simulator simply stopping. The required response and its limits must be defined before qualification.

Passing a family requires its declared representative and boundary cases, unchanged operating/numerical limits, retained source/settings identity and evidence. Record tracking, overshoot, settling, clearance, observability, current, voltage, losses and recovery as applicable. Add cases when a new mechanism or failure reveals a coverage gap; merge redundant cases rather than maintaining a growing session-by-session catalog.

## Neighbours and deferred scope

Do an early static screening of a resting neighbour's magnetic attraction/torque and Hall-field interference over relevant separations and orientations, including the powered-off touching-base condition in the product vision. This can expose a design conflict before investing in the full single-piece campaign; it does not qualify interacting motion.

Keep the main controller investigation single-piece until representative ordinary and combined flight works with bounded disturbances. Then add a moving piece beside a resting neighbour, multiple moving pieces, shared-coil allocation and simultaneous sliding reset. High ballistic jumps and catching are deferred feasibility work outside this checklist; do not treat the airborne gap model as supporting them.

## Keep execution affordable

Screen planned paths for geometry, sensing and force/torque authority before expensive feedback runs. Use saved poses and dense trajectories for diagnostics and performance comparisons. Qualify faster backends against the retained reference with unchanged accuracy budgets before using them for the campaign. Validate newly introduced mechanisms before interpreting their control results.

Use short decisive cases first, followed by the necessary settling, repeated-cycle and complete-move intervals. Reuse compatible checkpoints only when the intended initial physical, controller and sensor state is the same; explicit branch/extension support must preserve source identity, hardware clocks and random state. Do not splice independent passing phases into a claim of continuous success. Stop automatically on a failed gate, preserve evidence, and wait for a launched process to finish without periodic progress inspections.
