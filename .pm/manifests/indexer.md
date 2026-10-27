# Indexer — Integration Manifest

Class: `org.firstinspires.ftc.teamcode.subsystems.Indexer`
(`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Indexer.java`)

Public API: `enum State { OFF, INTAKING, FEEDING, REVERSING }`, `setState(State)`, `getState()`, `periodic()`.
Initial state: `OFF`. Motor setup in constructor: direction from constants, zero-power BRAKE, `RUN_WITHOUT_ENCODER`.

## 1. Instantiation

Import:
```java
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
```
Field + `init()`:
```java
private Indexer indexer;
// in init():
indexer = new Indexer(hardwareMap, RoboConstants.INDEXER_MOTOR_NAME, telemetry);
```
Every loop (TeleOp `loop()` and the auto sequencer loop), after the coordination logic sets the state:
```java
indexer.periodic();
```
Recommended in `stop()`: `indexer.setState(Indexer.State.OFF); indexer.periodic();`

## 2. RoboConstants entries (add under `// ===== Indexer =====`)

Required import in RoboConstants: `import com.qualcomm.robotcore.hardware.DcMotorSimple;`

| Type | Name | Default | Tunable? |
|---|---|---|---|
| `public static final String` | `INDEXER_MOTOR_NAME` | `"indexer"` | No (HW map name, EH M1) |
| `public static final DcMotorSimple.Direction` | `INDEXER_DIRECTION` | `DcMotorSimple.Direction.FORWARD` | No, it's only applied in the constructor. Verify that positive power moves balls toward the shooter |
| `public static double` | `INDEXER_INTAKE_POWER` | `0.6` | Yes (read every loop) |
| `public static double` | `INDEXER_FEED_POWER` | `1.0` | Yes |
| `public static double` | `INDEXER_REVERSE_POWER` | `-1.0` | Yes |

```java
// ===== Indexer =====
public static final String INDEXER_MOTOR_NAME = "indexer";
public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.FORWARD;
public static double INDEXER_INTAKE_POWER = 0.6;
public static double INDEXER_FEED_POWER = 1.0;
public static double INDEXER_REVERSE_POWER = -1.0;
```

## 3. Gamepad bindings (gamepad1, through `GamepadEx`)

These are evaluated in the OpMode's Coordination block, highest priority first. The Indexer only receives `setState(...)`.

| Control | Indexer state |
|---|---|
| Left bumper (hold) | `REVERSING` (priority 1) |
| Right trigger (hold, > 0.3) | `OFF` until Flywheel first reports `AT_SPEED`, then `FEEDING`, latched until release (priority 2) |
| Left trigger (hold, > 0.3) | `INTAKING` (priority 3) |
| A (press, toggles Match Loader) | While Match Loader is `UP`: `INTAKING` (priority 3) |
| Nothing above active | `OFF` (priority 5) |

## 4. OpModes to wire into

- `JoJaTeleOp`: full Coordination as above, and `indexer.periodic()` once per loop.
- `JoJaAutoNear` and `JoJaAutoFar`: the `shoot(seconds)` step sets `FEEDING` once `flywheel.isAtSpeed()` is true (with Stopper `OPEN` and Intake `INTAKING`), then `OFF` when the step ends. All other steps leave it `OFF`. Call `periodic()` every loop.

## 5. Cross-subsystem signals (consumed by the OpMode, not the Indexer)

- **Flywheel:** `isAtSpeed()` gates `FEEDING`. The first `true` during a Shoot hold latches feeding until release.
- **Match Loader:** `getState() == UP` (or the equivalent getter) means the Indexer goes to `INTAKING`.
- The Indexer exposes `getState()` for telemetry and debugging. No other subsystem needs to read it.
