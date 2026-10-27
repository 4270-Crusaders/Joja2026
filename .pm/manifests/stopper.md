# Integration Manifest: Stopper

Class: `org.firstinspires.ftc.teamcode.subsystems.Stopper`
File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Stopper.java`

## Public API

| Member | Purpose |
|---|---|
| `enum State { CLOSED, OPEN }` | Stopper states (initial: `CLOSED`) |
| `void open()` | Request `OPEN` (written to the servo on the next `periodic()`) |
| `void close()` | Request `CLOSED` |
| `State getState()` | Current requested state |
| `boolean isOpen()` | `getState() == State.OPEN` |
| `void periodic()` | Writes the servo position and the telemetry block. Call it once per loop. |

The constructor also commands `STOPPER_CLOSED_POS` once, so preloads are held from init (same pattern as the reference `Indexer`).

## Instantiation (in `init()` of each OpMode)

```java
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

private Stopper stopper;

stopper = new Stopper(hardwareMap, RoboConstants.STOPPER_SERVO_NAME, telemetry);
```

Each loop: `stopper.periodic();`

## Required RoboConstants entries

Add under `// ====== Stopper ======`:

```java
// ====== Stopper ======
public static final String STOPPER_SERVO_NAME = "stopper";   // CH Servo 0
public static double STOPPER_CLOSED_POS = 0.0;               // tune
public static double STOPPER_OPEN_POS = 0.5;                 // tune
```

| Type | Name | Default | Tunable |
|---|---|---|---|
| `String` (`public static final`) | `STOPPER_SERVO_NAME` | `"stopper"` | No |
| `double` (`public static`) | `STOPPER_CLOSED_POS` | `0.0` | Yes |
| `double` (`public static`) | `STOPPER_OPEN_POS` | `0.5` | Yes |

## Gamepad bindings (gamepad1, TeleOp)

| Control | Stopper action |
|---|---|
| Right trigger held (> 0.3), Shoot | `close()` until Flywheel first reports `AT_SPEED`, then `open()`. **Latched** for the rest of the hold (an RPM dip does not re-close it). |
| Right trigger released | `close()`, clear the shoot latch |
| Left trigger held (> 0.3), Intake | `close()` |
| Left bumper held, Reverse | `close()` (top priority) |
| A toggled so Match Loader is `UP` | `close()` (same rule as Intake) |
| Nothing held (idle) | `close()` |

Priority: Reverse > Shoot > Intake / Match Loader UP > idle. In practice `open()` is only called from the latched Shoot branch, and every other branch calls `close()`.

Suggested OpMode snippet (latch lives in the OpMode):

```java
// field: private boolean shootLatched = false;
boolean reverse = driver.getButton(GamepadKeys.Button.LEFT_BUMPER);
boolean shoot = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3;

if (reverse) {
    stopper.close();
    // (shoot latch is not cleared here unless shoot is released; see flag below)
} else if (shoot) {
    if (flywheel.isAtSpeed()) shootLatched = true;
    if (shootLatched) stopper.open(); else stopper.close();
} else {
    stopper.close();
}
if (!shoot) shootLatched = false;
```

## OpModes to wire into

- `JoJaTeleOp`: instantiate, bindings above, and call `periodic()` every loop.
- `JoJaAutoNear`: instantiate. The `shoot(seconds)` step waits for `flywheel.isAtSpeed()`, then `stopper.open()` for the duration, then `stopper.close()` at the end of the step. Call `periodic()` every loop.
- `JoJaAutoFar`: same as Near.

## Cross-subsystem signals

| Needs | From | Signal |
|---|---|---|
| Gate on the flywheel being at speed before opening | Flywheel | `flywheel.isAtSpeed()` (boolean); OpMode-side only |
| Provides | To | Signal |
| Stopper state, for telemetry or other gating if needed | OpMode | `stopper.isOpen()` / `stopper.getState()` |

The Stopper class itself has no references to other subsystems.

## Flags / ambiguities

1. **Reverse pressed during a latched shoot:** Reverse has higher priority, so the stopper closes. The backlog does not say whether the shoot latch should clear when Reverse interrupts a held Shoot. If it doesn't, releasing Reverse while Shoot is still held reopens the stopper immediately, even if the flywheel is not at speed. Suggest clearing the latch whenever Reverse is active. Integrator/PM to decide.
2. **Init-time servo write:** the constructor commands CLOSED once (as the reference Indexer does). This moves the servo during init. That is intended, to retain preloads, but the team should know the servo moves at init.
3. **Servo direction / range:** not specified in the backlog, so the SDK default (FORWARD, full range) is used. If the servo is mounted mirrored, swap the two position constants rather than adding a direction constant.
4. Positions 0.0 / 0.5 are placeholders and must be tuned via Panels.
