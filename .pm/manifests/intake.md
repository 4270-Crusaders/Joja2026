# Intake — Integration Manifest

Class: `org.firstinspires.ftc.teamcode.subsystems.Intake` (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Intake.java`)

Public API:
- `enum Intake.State { OFF, INTAKING, REVERSING }`
- `void setState(Intake.State state)` / `Intake.State getState()`
- `void periodic()` writes motor power + telemetry block. Initial state is `OFF`.

## 1. Instantiation (OpMode `init()`)

```java
import org.firstinspires.ftc.teamcode.subsystems.Intake;

private Intake intake;

intake = new Intake(hardwareMap, RoboConstants.INTAKE_MOTOR_NAME, telemetry);
```

In `loop()`, after the coordination logic has set the state: `intake.periodic();` (once per loop).
Stop / end of auto: `intake.setState(Intake.State.OFF); intake.periodic();`

## 2. Required RoboConstants entries

Add under a `// ====== Intake ======` block. `INTAKE_DIRECTION` needs `import com.qualcomm.robotcore.hardware.DcMotorSimple;` in `RoboConstants.java`.

| Type | Name | Default | Tunable? | Notes |
|---|---|---|---|---|
| `String` | `INTAKE_MOTOR_NAME` | `"intake"` | No: `public static final` | EH Motor 0 |
| `DcMotorSimple.Direction` | `INTAKE_DIRECTION` | `DcMotorSimple.Direction.FORWARD` | No: `public static final` (one-line flip) | Only read in the constructor. Verify that positive pulls balls in. |
| `double` | `INTAKE_POWER` | `1.0` | Yes: `public static` | Read each `periodic()`, so live Panels edits apply |
| `double` | `INTAKE_REVERSE_POWER` | `-1.0` | Yes: `public static` | Read each `periodic()` |

```java
// ====== Intake ======
public static final String INTAKE_MOTOR_NAME = "intake";
public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
public static double INTAKE_POWER = 1.0;
public static double INTAKE_REVERSE_POWER = -1.0;
```

## 3. Gamepad bindings (gamepad1, from control-mapping.md)

| Control | Intake state |
|---|---|
| Left bumper (hold) | `REVERSING` (highest priority) |
| Right trigger (hold, > 0.3) | `OFF` until Flywheel first reaches `AT_SPEED`, then `INTAKING`, latched until release |
| Left trigger (hold, > 0.3) | `INTAKING` |
| A (press, toggles Match Loader) | `INTAKING` while Match Loader is `UP` |
| none of the above | `OFF` |

Suggested OpMode snippet (priority order per backlog Coordination; `shootLatched` is owned by the OpMode's shoot logic):

```java
if (driver.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
    intake.setState(Intake.State.REVERSING);
} else if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3) {
    intake.setState(shootLatched ? Intake.State.INTAKING : Intake.State.OFF);
} else if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3
        || matchLoader.getState() == MatchLoader.State.UP) {
    intake.setState(Intake.State.INTAKING);
} else {
    intake.setState(Intake.State.OFF);
}
```

## 4. OpModes to wire into

- `JoJaTeleOp` (all bindings above).
- `JoJaAutoNear` and `JoJaAutoFar`: instantiate and call `periodic()` each loop. The `shoot(seconds)` step should set `INTAKING` once the flywheel is `AT_SPEED` (same as TeleOp shoot), and `OFF` when the step ends. Set `OFF` when the sequence finishes.

## 5. Cross-subsystem signals

The Intake subsystem itself depends on nothing. The OpMode needs these to decide the Intake state:
- **Flywheel:** `isAtSpeed()` drives the shoot latch. Intake goes `INTAKING` only after the first `AT_SPEED` in a shoot hold.
- **MatchLoader:** its state getter, e.g. `getState() == MatchLoader.State.UP` (exact name depends on the Match Loader head's API). While it is UP, Intake runs `INTAKING`.
- Indexer and Stopper are set alongside the Intake in the same branches. Intake does not read them.

## Flags / ambiguities

1. **Auto `shoot()` step and intake:** the backlog Auto section only says "opens stopper + feeds". I assumed the intake also runs `INTAKING` during the auto shoot, mirroring TeleOp. Confirm, or leave it `OFF` in auto.
2. **Match Loader UP while Shoot is held:** Shoot takes priority, so the intake stays `OFF` until `AT_SPEED` even when the loader is UP. This follows the stated priority, but the intake briefly pauses at the start of a shoot hold.
3. **`INTAKE_DIRECTION` is `final`:** it is only applied at construction, so making it Panels-tunable would do nothing live. Integrator may make it non-final if the team prefers, but it still only takes effect on re-init.
