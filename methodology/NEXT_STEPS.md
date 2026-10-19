# Next steps

Complete the virtual design before manufacturing or physical tests. Use the existing CAD, MuJoCo, magnetic, electrical and sensing implementations. The [simulator reference](docs/SIMULATOR.md) defines numerical verification; the [single-piece checklist](docs/SINGLE_PIECE_TESTS.md) defines flight-test coverage; [MODELING_APPROACH.md](docs/MODELING_APPROACH.md#physical-model-verification-gate) retains measured evidence and limitations.

## Ready for the first flight campaign

The optimized simulator has passed the [production readiness gate](results/local_motion/optimized/production_readiness/summary.json): fresh source-checked fields, integrated force/torque comparisons, a short sensor-feedback movement, exact fresh-process restart during sensor acquisition, complete recordings and independent sensing checks. Focused tests cover deadline/failure stops and invalid cache/runtime inputs. The [performance reference](docs/MODELING_APPROACH.md#next-performance-work) owns the measured results and scope.

**Stop here to agree on the first flight campaign.** Use the integrated coil-cache runner and its current production artifacts. Start with the basic single-piece flight families below, keeping each experiment tied to an unresolved control question and an acceptance rule. Do not repeat the completed optimization comparison unless new evidence or a changed mechanism requires it.

Simulation-ready means ready to run experiments within the verified single-piece airborne scope. It does not establish disturbance recovery, all movements, contact or multi-piece control. Diagnose failures from recordings first; extend verification when entering unsupported conditions.

## Flight and design qualification

1. **Establish basic single-piece flight.** Follow SP01–SP05 in the checklist: settling, translation/braking, vertical movement, tilt and continuous rotation. Extend the shared reference generator without animation-specific controllers. Introduce the asymmetric knight alongside the king once basic references work. Keep existing noise enabled and add bounded disturbance/variation checks as each family matures. Perform an early static neighbour force/sensing screen before committing to the full campaign; it does not require a multi-piece dynamic controller.
2. **Complete ordinary chess movement.** Follow SP06–SP09: controlled coil/sensor handoffs, verified contact, takeoff, landing and continuous complete moves. Preserve failed contact evidence and unchanged limits until replacement verification passes. Resolve or bound inductance/PWM interference, current-readout timing and construction variation before adopting final hardware targets.
3. **Qualify combined movement and robustness.** Follow SP10–SP12 and the checklist's cross-cutting coverage: curved paths, simultaneous movement/rotation/tilt, oscillation and coordinated dance. Expand disturbance recovery, piece coverage, construction variation and defined fault responses. Revisit numerical verification for new mechanisms or unsupported domains, without repeating unrelated verification for every animation.
4. **Extend to the whole board.** Add neighbours, edges, capture buffers, shared-coil allocation and simultaneous sliding reset. Integrate power distribution, processing, communication, enclosure, cooling and the real-component BOM. Evaluate product workloads, manufacturing variation and faults, keeping single-piece levitation distinct from simultaneous sliding reset.
5. **Review the integrated virtual design.** Require reproducible sensor-based control, numerical verification for the intended use, credible uncertainty ranges and consistent mechanical, electrical and thermal evidence. Only then plan manufacturing and physical validation.

Develop thermal analysis alongside electronics and mechanics, recording currents, voltages and losses and feeding relevant temperature effects back into the model. Resolve winding fit and assembly tolerances before fixing hardware targets.

Before an expensive run, identify the unresolved property, acceptance rule and expected decision. Inspect existing processes/evidence, prefer recorded-data diagnosis, and preserve source/settings provenance, automatic stops and checkpoints. Wait for launched batches to finish rather than periodically inspecting unchanged progress. Never treat process completion as a passed simulation or relax limits merely to obtain a pass.

Keep only remaining work here. Quantitative settings belong in `Inputs`, `Fixed` and `Constants` in [model.py](../model.py), with descriptive comments. Follow the [organization rules](PROJECT_ORGANIZATION.md); reuse existing calculation owners and prune superseded guidance as work proceeds.
