# Flywheel — Integration Manifest

Class: `org.firstinspires.ftc.teamcode.subsystems.Flywheel` (extends FTCLib `SubsystemBase`)
File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Flywheel.java`

## 1. Instantiation (in `init()` of each OpMode)

```java
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

private Flywheel flywheel;

flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME, telemetry);
```

Call `flywheel.periodic();` once per `loop()` (after coordination logic has called `spinUp()`/`stop()` for this loop). Initial state is `OFF`; no other init call needed. Call `flywheel.stop(); flywheel.periodic();` in `stop()` if the OpMode overrides it (optional, the SDK zeroes motors anyway).

## 2. Required RoboConstants entries

Add a `// ====== Flywheel ======` block. Requires `import com.qualcomm.robotcore.hardware.DcMotorSimple;` in RoboConstants. `LOOP_HZ` already exists.

| Type | Name | Default | Tunable (Panels) | Notes |
|---|---|---|---|---|
| `public static final String` | `FLYWHEEL_LEFT_MOTOR_NAME` | `"flywheelLeft"` | no | CH M2, verify against DS config |
| `public static final String` | `FLYWHEEL_RIGHT_MOTOR_NAME` | `"flywheelRight"` | no | EH M2, verify against DS config |
| `public static final DcMotorSimple.Direction` | `FLYWHEEL_LEFT_DIRECTION` | `DcMotorSimple.Direction.FORWARD` | no | Applied once in the constructor. Verify: positive power = shooting direction |
| `public static final DcMotorSimple.Direction` | `FLYWHEEL_RIGHT_DIRECTION` | `DcMotorSimple.Direction.REVERSE` | no | Opposite of A (motors facing each other on one shaft). Verify |
| `public static final int` | `FLYWHEEL_TPR` | `28` | no | Ticks/rev at the motor (5203 bare) |
| `public static double` | `FLYWHEEL_GEAR_RATIO` | `1.0` | yes | **Placeholder.** Motor revs per wheel rev; wheel RPM = motor RPM / ratio. Must be > 0 |
| `public static double` | `FLYWHEEL_TARGET_RPM` | `3000.0` | yes | **Placeholder.** Wheel RPM; read every loop so live edits apply |
| `public static double` | `FLYWHEEL_RPM_TOLERANCE` | `50.0` | yes | `AT_SPEED` when abs(target - rpm) <= this |
| `public static double` | `FLYWHEEL_kP` | `0.0005` | yes | **Placeholder.** Power per RPM of error (100 RPM err gives 0.05 power) |
| `public static double` | `FLYWHEEL_kI` | `0.0` | yes | **Placeholder.** Start at 0 |
| `public static double` | `FLYWHEEL_kD` | `0.0` | yes | **Placeholder.** Start at 0 |
| `public static double` | `FLYWHEEL_kF` | `0.00017` | yes | **Placeholder.** Power per target wheel RPM (about 1/6000 for a 6000 RPM free-speed motor at ratio 1.0). Re-tune when the gear ratio is known (roughly `FLYWHEEL_GEAR_RATIO / 6000`, then raise until steady-state RPM is near target) |

Note on kF units: per the backlog, feedforward is `kF * targetRpm` (wheel RPM), not the reference repo's `kF * ticks/s`.

## 3. Gamepad bindings (gamepad1, from control-mapping.md)

| Control | Flywheel action |
|---|---|
| Right bumper (hold) | `flywheel.spinUp()` while held |
| Right trigger (hold, > 0.3) | `flywheel.spinUp()` while held (part of Shoot) |
| Neither held | `flywheel.stop()` |

Suggested TeleOp code (Coordination rule 4; independent of rules 1/2/3/5):

```java
boolean spinHeld  = gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
boolean shootHeld = gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3;
if (spinHeld || shootHeld) flywheel.spinUp(); else flywheel.stop();
```

(`gp1` is the FTCLib `GamepadEx`; call `gp1.readButtons()` each loop per the OpMode's convention.) `spinUp()` and `stop()` are idempotent and safe to call every loop.

## 4. OpModes to wire into

- `JoJaTeleOp`: bindings above; telemetry block comes from `periodic()`.
- `JoJaAutoNear` and `JoJaAutoFar`: sequencer step `spinUp()` calls `flywheel.spinUp()`; step "Stop flywheel" calls `flywheel.stop()`; step `shoot(seconds)` waits on `flywheel.isAtSpeed()`. `flywheel.periodic()` must run every loop for the whole auto (the PIDF only runs inside `periodic()`).

## 5. Cross-subsystem signals

Flywheel consumes nothing from other subsystems. It provides:

| Signal | Type | Consumer (in the OpMode) |
|---|---|---|
| `isAtSpeed()` | `boolean` | Shoot logic: gates Stopper `open()`, Indexer `FEEDING`, Intake `INTAKING`. The OpMode must **latch** the first `true` for the rest of the Shoot hold (Flywheel itself does not latch; it drops back to `SPINNING_UP` on an RPM dip) |
| `getState()` | `Flywheel.State` (`OFF`, `SPINNING_UP`, `AT_SPEED`) | Telemetry / auto sequencer |
| `getRpm()` | `double` (wheel RPM) | Telemetry / tuning |
| `getTargetRpm()` | `double` | Telemetry (returns `RoboConstants.FLYWHEEL_TARGET_RPM`) |

## Flags

- Gear ratio, target RPM and all four gains are placeholders; tune on the robot.
- No integral anti-windup (backlog doesn't specify one). Harmless at the default `kI = 0`; if kI is used, the integral only resets on `stop()`.
- `AT_SPEED` can be reported on a single loop's RPM sample; there is no settle-time debounce (not in backlog). The shoot latch means one noisy in-tolerance sample opens the stopper. Add a debounce later if that's a problem.
- `FLYWHEEL_LEFT_DIRECTION` / `FLYWHEEL_RIGHT_DIRECTION` are final because it's applied only in the constructor; live Panels edits would not take effect anyway.
