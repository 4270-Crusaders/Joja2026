# Integration Manifest — Match Loader

Class: `org.firstinspires.ftc.teamcode.subsystems.MatchLoader` (extends FTCLib `SubsystemBase`)
File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MatchLoader.java`

## Public API

| Member | Purpose |
|---|---|
| `enum State { DOWN, UP }` | Subsystem state. Initial: `DOWN`. |
| `up()` / `down()` / `toggle()` | Store the state only. No hardware writes. |
| `getState()` | Returns the current `State`. |
| `isUp()` | `true` when `State.UP`. This is the signal the Coordination logic reads. |
| `periodic()` | Writes the servo position for the current state and adds the `====== Match Loader ======` telemetry block. Call once per loop. |

## Instantiation

Imports:
```java
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
```

Field and `init()`:
```java
private MatchLoader matchLoader;
// in init():
matchLoader = new MatchLoader(hardwareMap, RoboConstants.MATCH_LOADER_SERVO_NAME, telemetry);
```

Every loop (`loop()`, plus the auto sequencer's loop):
```java
matchLoader.periodic();
```

## Required RoboConstants entries

Add under a `// ====== Match Loader ======` block:

| Type | Name | Default | Tunable? |
|---|---|---|---|
| `public static final String` | `MATCH_LOADER_SERVO_NAME` | `"matchLoader"` | No (hardware name, CH Servo 1) |
| `public static double` | `MATCH_LOADER_DOWN_POS` | `0.0` | Yes (tune on robot) |
| `public static double` | `MATCH_LOADER_UP_POS` | `0.5` | Yes (tune on robot) |

```java
// ====== Match Loader ======
public static final String MATCH_LOADER_SERVO_NAME = "matchLoader";
public static double MATCH_LOADER_DOWN_POS = 0.0;
public static double MATCH_LOADER_UP_POS = 0.5;
```

## Gamepad bindings (gamepad1, from control-mapping.md)

| Control | Action |
|---|---|
| A (press, edge-detected) | `matchLoader.toggle()` (`DOWN` <-> `UP`) |

```java
// gamepadEx = new GamepadEx(gamepad1);
gamepadEx.readButtons();  // once per loop, before the checks
if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
    matchLoader.toggle();
}
```
Imports: `com.arcrobotics.ftclib.gamepad.GamepadEx`, `com.arcrobotics.ftclib.gamepad.GamepadKeys`.

## OpModes to wire into

| OpMode | Wiring |
|---|---|
| `JoJaTeleOp` ("JoJa Tele") | Instantiate, A toggles it, call `periodic()` each loop, and use `isUp()` in Coordination rule 3. |
| `JoJaAutoNear` | Instantiate and call `periodic()` each loop. It has to exist so the shared sequencer's `matchLoader(UP/DOWN)` step type works, even though the current step list doesn't use it. |
| `JoJaAutoFar` | Same as `JoJaAutoNear`. |

Auto sequencer step `matchLoader(UP/DOWN)` should call `matchLoader.up()` or `matchLoader.down()` and finish right away. The subsystem has no `isBusy()`, because a servo gives no feedback. If the plate needs time to travel, put a `wait(seconds)` step after it.

## Cross-subsystem signals

- **Provides:** `matchLoader.isUp()`, read by the TeleOp Coordination logic.
- **Consumed by the OpMode (not by this subsystem):** Coordination rule 3. If Intake is held OR `matchLoader.isUp()`, then Intake `INTAKING`, Indexer `INTAKING`, Stopper `CLOSED`. Reverse (rule 1) and Shoot (rule 2) still take priority over Match Loader UP.
- **Needs from other subsystems:** none.

## Notes for the integrator

- The servo is not written until the first `periodic()` call. To hold the plate DOWN during init, also call `matchLoader.periodic()` in `init_loop()`. Otherwise the servo stays unpowered until Start.
- Panels live-tuning of `MATCH_LOADER_*_POS` only works if `RoboConstants` carries Panels' `@Configurable` annotation (or the equivalent). The current `RoboConstants` doesn't have it yet.
