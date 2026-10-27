# JoJa Control Mapping (TeleOp — single driver)

Priority when several are held: **Reverse > Shoot > Intake / Match Loader UP > idle** (see Coordination in `backlog.md`).

## gamepad1

| Control | Action | Subsystem | Resulting State/Behavior |
|---|---|---|---|
| Left stick Y | Drive forward/back | Drivetrain | `MANUAL`, robot-centric, full speed |
| Left stick X | Strafe left/right | Drivetrain | `MANUAL` |
| Right stick X | Turn | Drivetrain | `MANUAL` |
| Right trigger (hold, > 0.3) | Shoot | Flywheel, Stopper, Indexer, Intake | Flywheel spins up automatically → once `AT_SPEED`: Stopper `OPEN`, Indexer `FEEDING`, Intake `INTAKING` (latched until release). Release → Stopper `CLOSED`, Indexer/Intake `OFF` |
| Right bumper (hold) | Spin up flywheel | Flywheel | `SPINNING_UP` → `AT_SPEED`; release → `OFF` (unless Shoot held) |
| Left trigger (hold, > 0.3) | Intake | Intake, Indexer, Stopper | Intake `INTAKING`, Indexer `INTAKING`, Stopper `CLOSED` |
| Left bumper (hold) | Reverse / eject | Intake, Indexer, Stopper | Intake `REVERSING`, Indexer `REVERSING`, Stopper `CLOSED` |
| A (press) | Toggle match loader | Match Loader (+ Intake, Indexer) | `DOWN` ↔ `UP`; while `UP`, Intake + Indexer run `INTAKING` automatically |

Unused: B, X, Y, D-pad, stick buttons, back, start.

### Init only (Autonomous OpModes)

| Control | Action |
|---|---|
| X | Select Blue alliance |
| B | Select Red alliance (mirrors strafes and turns) |

## gamepad2

Not used (single driver).
