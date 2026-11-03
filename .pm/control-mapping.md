# JoJa Control Mapping (TeleOp — single driver)

Priority when several are held: **Reverse > Intake > idle**. Flywheel and stopper are independent.

## gamepad1

Shooting is **manual**: the driver spins up and opens the stopper whenever they want. Nothing waits for the flywheel.

| Control | Action | Subsystem | Resulting State/Behavior |
|---|---|---|---|
| Left stick Y | Drive forward/back | Drivetrain | `MANUAL`, robot-centric, full speed |
| Left stick X | Strafe left/right | Drivetrain | `MANUAL` |
| Right stick X | Turn | Drivetrain | `MANUAL` |
| Right trigger (hold, > 0.3) | Spin flywheel + indexer | Flywheel, Indexer | Per-side PIDF (boost) to `FLYWHEEL_TARGET_RPM`. Indexer `FEEDING` if stopper open, `INTAKING` if closed. Intake NOT run. Keeps going 0.5 s after release |
| Right bumper (press) | Toggle stopper | Stopper | `CLOSED` (0.25) ↔ `OPEN` (0.05). Starts `CLOSED` |
| Left trigger (hold, > 0.3) | Intake / feed | Intake, Indexer, Stopper | Flywheel off: press → stopper `CLOSED`, Intake + Indexer `INTAKING`. Flywheel on + stopper open: Indexer `FEEDING` (shoots), stopper not touched |
| B / X (hold either) | Outtake (reverse) | Intake, Indexer | Intake `REVERSING`, Indexer `REVERSING` (stopper unchanged) |
| Left bumper / L1 (press) | Toggle match loader | Match Loader | Toggle only. Starts `UP` (0.64); each press `DOWN` (0.29) ↔ `UP` |
| Y (press) | Match loader middle ↔ up | Match Loader | From `MIDDLE` → `UP`; otherwise → `MIDDLE` (pollen only, no nectar) |
| A (press) | Match loader middle ↔ down | Match Loader | From `MIDDLE` → `DOWN`; otherwise → `MIDDLE`. L1 from `MIDDLE` goes `UP` |

**To shoot:** hold right trigger (flywheel + indexer) → press right bumper to open the stopper → balls feed (hold left trigger too to run the intake roller) → press right bumper again to close. Flywheel keeps spinning 0.5 s after you let go of RT.

**While the stopper is open:** a side more than 30 RPM below target gets full power (shot recovery), and the indexer pauses while either side is more than 100 RPM below target, then feeds again once both are back within 40 RPM (feed gate). So each ball hits a wheel that is back at full speed.

Unused: D-pad, stick buttons, back, start. (Match loader zero offset is set in JoJa Hardware Test, not here.)

### Init only (Autonomous OpModes)

| Control | Action |
|---|---|
| D-pad left | Select Blue alliance |
| D-pad right | Select Red alliance (mirrors poses) |

## gamepad2

Not used (single driver).
