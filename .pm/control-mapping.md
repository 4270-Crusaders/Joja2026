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
| B (hold) | Outtake (reverse) | Intake, Indexer | Intake `REVERSING`, Indexer `REVERSING` (stopper unchanged) |
| A (press) | Set position to (0,0) | Odometry | Pinpoint X/Y = 0, heading kept (Pedro field: (0,0) bottom-left). 2026-11-09: was X, then moved to A |
| X, Y | (nothing) | – | 2026-11-09: Y / A match loader MIDDLE buttons removed (MIDDLE code kept in MatchLoader) |
| Left bumper / L1 (press) | Toggle match loader | Match Loader | Toggle only. Starts `UP` (0.64); each press `DOWN` (0.29) ↔ `UP` |

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


**Auto shoot (2026-11-09):** holding RT spins up the flywheel; once both sides are within `AUTO_SHOOT_READY_RPM` of target for `AUTO_SHOOT_READY_S`, the stopper opens by itself. Each ball is detected as an RPM dip; once no new dip for `AUTO_SHOOT_EMPTY_S` (or no ball within `AUTO_SHOOT_NO_BALL_S`, or `AUTO_SHOOT_MAX_S`), the stopper closes. Once per RT press. R1 still toggles the stopper by hand and cancels auto until RT is released. Off switch: `AUTO_SHOOT_ENABLED`.

**Stopper (2026-11-09):** whenever RT is not held, the stopper is forced CLOSED (auto shoot or R1 can only open it while RT is held).
