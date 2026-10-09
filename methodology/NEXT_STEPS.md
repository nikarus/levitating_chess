# Next steps

Develop mechanics, electronics, control and thermal design together through small, testable prototypes. Use the existing model and adopted targets as the starting point, improving them when construction details or measurements justify a change.

1. **Define a buildable prototype.** Develop the coil/magnet assembly, representative current driver and sensing arrangement together. Check winding manufacture and physical fit, identify candidate real components, and define their interfaces. Maintain simple loss and temperature estimates, with a plausible heat path and physical space for cooling. Prepare a fixture and measurement procedure.

2. **Build, measure and revise.** Characterize the coil's resistance, inductance, force and heating. Measure driver response, losses and sensing performance. Compare results with predictions and update the existing model. Bench equipment can support early coil measurements while the driver is being developed. Use detailed thermal or electromagnetic simulation to answer specific questions exposed by the design or measurements.

3. **Demonstrate local levitation.** Combine the coils, driver, sensing and controller. Verify king hover, complete movements, tilt/yaw and coil handoffs. Check nearby pieces, repeated-use heating and relevant fault responses. Iterate the assembly and model until this representative system works within its intended operating conditions.

4. **Scale and validate the board.** Resolve board-wide sensing and communication, power distribution, cooling, edge coverage and simultaneous sliding reset. Evaluate single-piece levitation, repeated local activity and full-set reset workloads. Complete the production layout and BOM once the system checks support them, then verify the integrated board against the product tests.

Thermal assessment continues throughout these steps. The existing thermal failures remain open issues to investigate before committing to the full-board construction. Detailed thermal optimization can follow functional experiments; temporary cooling or limited-duration tests must be recorded as prototype limitations. Measurements determine where more detailed modeling is worth the effort.

**Immediate deliverable:** a coherent prototype design with an assembly drawing, candidate component list, driver/sensing interfaces, a simple thermal feasibility estimate and a combined electrical, magnetic and thermal measurement plan.

Keep quantitative settings in `Inputs`, `Fixed` and `Constants` in [model.py](../model.py), with descriptive comments. Reuse existing calculations and keep supporting work under `methodology/`.

See [MODELING_APPROACH.md](MODELING_APPROACH.md) for tool choices and modeling scope, and [QUALIFICATION.md](QUALIFICATION.md) for the current simulation evidence and limitations.
