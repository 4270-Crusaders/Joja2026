# Integration Manifest: Drivetrain

File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drivetrain.java`

## Instantiation (OpMode `init()`)

```java
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

drivetrain = new Drivetrain(hardwareMap,
        RoboConstants.DRIVE_LEFT_FRONT_NAME,
        RoboConstants.DRIVE_LEFT_BACK_NAME,
        RoboConstants.DRIVE_RIGHT_FRONT_NAME,
        RoboConstants.DRIVE_RIGHT_BACK_NAME,
        telemetry);
```

The battery voltage sensor is pulled internally from `hardwareMap.voltageSensor.iterator().next()`, so it has no constant. Initial state is `IDLE` (powers 0).

Call `drivetrain.periodic();` once per loop (TeleOp and Auto), after the drive command for that loop.

## RoboConstants entries (`// ====== Drivetrain ======` block)

Requires `import com.qualcomm.robotcore.hardware.DcMotorSimple;` in RoboConstants.

| Type | Name | Default | Tunable |
|---|---|---|---|
| `String` | `DRIVE_LEFT_FRONT_NAME` | `"leftFront"` | No (`public static final`) |
| `String` | `DRIVE_LEFT_BACK_NAME` | `"leftBack"` | No (`public static final`) |
| `String` | `DRIVE_RIGHT_FRONT_NAME` | `"rightFront"` | No (`public static final`) |
| `String` | `DRIVE_RIGHT_BACK_NAME` | `"rightBack"` | No (`public static final`) |
| `DcMotorSimple.Direction` | `DRIVE_LEFT_FRONT_DIRECTION` | `DcMotorSimple.Direction.REVERSE` | Yes (`public static`) |
| `DcMotorSimple.Direction` | `DRIVE_LEFT_BACK_DIRECTION` | `DcMotorSimple.Direction.REVERSE` | Yes (`public static`) |
| `DcMotorSimple.Direction` | `DRIVE_RIGHT_FRONT_DIRECTION` | `DcMotorSimple.Direction.FORWARD` | Yes (`public static`) |
| `DcMotorSimple.Direction` | `DRIVE_RIGHT_BACK_DIRECTION` | `DcMotorSimple.Direction.FORWARD` | Yes (`public static`) |
| `double` | `DRIVE_NOMINAL_VOLTAGE` | `12.0` | Yes (`public static`) |
| `boolean` | `DRIVE_VOLTAGE_COMP_ENABLED` | `true` | Yes (`public static`) |

```java
// ====== Drivetrain ======
public static final String DRIVE_LEFT_FRONT_NAME  = "leftFront";
public static final String DRIVE_LEFT_BACK_NAME   = "leftBack";
public static final String DRIVE_RIGHT_FRONT_NAME = "rightFront";
public static final String DRIVE_RIGHT_BACK_NAME  = "rightBack";
public static DcMotorSimple.Direction DRIVE_LEFT_FRONT_DIRECTION  = DcMotorSimple.Direction.REVERSE;
public static DcMotorSimple.Direction DRIVE_LEFT_BACK_DIRECTION   = DcMotorSimple.Direction.REVERSE;
public static DcMotorSimple.Direction DRIVE_RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
public static DcMotorSimple.Direction DRIVE_RIGHT_BACK_DIRECTION  = DcMotorSimple.Direction.FORWARD;
public static double  DRIVE_NOMINAL_VOLTAGE      = 12.0;
public static boolean DRIVE_VOLTAGE_COMP_ENABLED = true;
```

Directions are only applied in the constructor, so a live Panels edit takes effect on the next init, not mid-run.

## Gamepad bindings (gamepad1, TeleOp, every loop)

| Control | Argument |
|---|---|
| Left stick Y | `fwd` (push up = forward, +) |
| Left stick X | `strafe` (push right = strafe right, +) |
| Right stick X | `turn` (push right = turn clockwise, +) |

```java
drivetrain.drive(driver.getLeftY(), driver.getLeftX(), driver.getRightX());
```

`driver` is the FTCLib `GamepadEx` wrapping gamepad1. `GamepadEx.getLeftY()` already flips the sign, so up is positive. If you read raw `gamepad1.left_stick_y` instead, negate it. There is no scaling, deadband or slow mode (full speed 1.0, per the backlog).

## OpModes to wire into

- **`JoJaTeleOp`**: instantiate it, call `drive(...)` every loop with the bindings above, then call `periodic()`.
- **`JoJaAutoNear`** and **`JoJaAutoFar`**: instantiate it and call `periodic()` every loop. The sequencer's `drive(fwd, strafe, turn, seconds)` step maps to:
  - On step start: `drivetrain.timedMove(fwd, strafe * m, turn * m, seconds)`, where `m = (alliance == RED) ? -1 : 1`.
  - Step is done when `!drivetrain.isBusy()`. The drivetrain goes to `IDLE` by itself.
  - On OpMode `stop()`, or when the sequence finishes: `drivetrain.stop()`.
  - Sign conventions for the auto steps: "drive backward" uses `fwd < 0`, "strafe out" uses `strafe` (+ is right), and "turn" uses `turn` (+ is clockwise).

## Public API

| Method | Effect |
|---|---|
| `drive(double fwd, double strafe, double turn)` | Moves to `MANUAL` and stores the inputs |
| `timedMove(double fwd, double strafe, double turn, double seconds)` | Moves to `TIMED_MOVE`, samples battery voltage once, resets the timer |
| `stop()` | Moves to `IDLE`, zeroes the inputs |
| `boolean isBusy()` | True while in `TIMED_MOVE` |
| `Drivetrain.State getState()` | Returns `IDLE`, `MANUAL` or `TIMED_MOVE` |
| `periodic()` | Ends a timed move, mixes, normalizes, writes powers, adds telemetry |

## Cross-subsystem signals

None. The Drivetrain doesn't consume or produce signals for other subsystems. The auto sequencer (OpMode) uses only `isBusy()`.

## Notes / flags

- **Voltage-comp clamp:** the wheel outputs are normalized by the largest magnitude rather than clamped one wheel at a time, so the move keeps its direction. For a single-axis move like the planned autos, this gives the same result as a plain ±1.0 clamp.
- **`isBusy()` timing:** `isBusy()` only goes false in the first `periodic()` after the duration has passed. The sequencer should check `isBusy()` after calling `drivetrain.periodic()` in the same loop, or accept up to one loop of extra drive time.
- **MANUAL holds the last command:** if the OpMode stops calling `drive()`, the drivetrain keeps the last stick values. TeleOp should call `drive()` every loop.
