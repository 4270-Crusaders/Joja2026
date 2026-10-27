# JoJa — BioBuzz (2026–27) Robot Backlog

| | |
|---|---|
| Team | 24056 |
| Season / game | BIOBUZZ (FTC 2026–27) |
| Robot | JoJa |
| Working project | `FTC BioBuzz\FtcRobotController` (official FIRST base, SDK 12.0) |
| Architecture reference | `FTC BioBuzz\24056-DECODE-Code-reference` (read-only) |
| Language | Java |

## Architecture (carried over from 24056-DECODE-Code)

- Package layout under `org.firstinspires.ftc.teamcode`: `RoboConstants.java`, `subsystems/`, `opmodes/`, `utils/`.
- **Departure (approved):** no `pedroPathing/` or `myLocalizer/` package — no Pedro Pathing, no localization.
- Every subsystem extends FTCLib `SubsystemBase`, constructor `(HardwareMap hMap, String... deviceNames, Telemetry telemetry)`, logs `RobotLog.i("[Name] Initialized.")`, and does its hardware writes + its own telemetry block in `periodic()`.
- All hardware map names and tuning values live in `RoboConstants`. Values that need tuning are `public static` (non-final) so Panels can edit them live.
- OpModes are iterative `OpMode`; gamepad wrapped in FTCLib `GamepadEx`. OpModes call every subsystem's `periodic()` once per loop.
- Cross-subsystem coordination (who runs the intake/indexer when) lives in the OpMode, not inside subsystems. Subsystems expose states; the OpMode decides.

## Hardware Summary

All hardware names and ports below were chosen by the PM as defaults — **verify on the robot and match the Driver Station configuration to these names.** Motor directions are defaults, also to verify.

| Device | Type | HW map name | Hub | Port |
|---|---|---|---|---|
| Left front drive | goBILDA 5203 435 RPM | `leftFront` | Control Hub | Motor 0 |
| Left back drive | goBILDA 5203 435 RPM | `leftBack` | Control Hub | Motor 1 |
| Right front drive | goBILDA 5203 435 RPM | `rightFront` | Control Hub | Motor 2 |
| Right back drive | goBILDA 5203 435 RPM | `rightBack` | Control Hub | Motor 3 |
| Intake | goBILDA 5203 1150 RPM | `intake` | Expansion Hub | Motor 0 |
| Indexer | goBILDA 5203 1150 RPM | `indexer` | Expansion Hub | Motor 1 |
| Flywheel A | goBILDA 5203 bare (6000 RPM, 28 TPR) | `flywheelA` | Expansion Hub | Motor 2 |
| Flywheel B | goBILDA 5203 bare (6000 RPM, 28 TPR) | `flywheelB` | Expansion Hub | Motor 3 |
| Ball stopper | goBILDA positional servo | `stopper` | Control Hub | Servo 0 |
| Match loader | goBILDA positional servo | `matchLoader` | Control Hub | Servo 1 |
| Battery voltage | Built-in hub voltage sensor | (first `VoltageSensor` in hardwareMap) | — | — |

No IMU, no camera/Limelight, no other sensors.

---

## Drivetrain

**Motors**

| Name | Model | Hub / Port | Direction (default) | Encoder |
|---|---|---|---|---|
| `leftFront` | 5203 435 RPM | CH M0 | REVERSE | not used |
| `leftBack` | 5203 435 RPM | CH M1 | REVERSE | not used |
| `rightFront` | 5203 435 RPM | CH M2 | FORWARD | not used |
| `rightBack` | 5203 435 RPM | CH M3 | FORWARD | not used |

- Mecanum, robot-centric, **full speed (1.0), no slow mode**.
- `RUN_WITHOUT_ENCODER`, zero-power BRAKE.
- Directions are constants in `RoboConstants` (one-line flip each).

**Sensors:** battery voltage only (for auto voltage compensation).

**States**

| State | Behavior |
|---|---|
| `IDLE` | All drive powers 0. Initial state. |
| `MANUAL` | TeleOp: standard mecanum mix from (forward, strafe, turn) stick inputs, normalized so no wheel exceeds 1.0. |
| `TIMED_MOVE` | Auto: hold a commanded (forward, strafe, turn) power for N seconds, then go to `IDLE` automatically. |

Transitions:
- `IDLE`/`TIMED_MOVE` → `MANUAL`: `drive(fwd, strafe, turn)` called (TeleOp).
- any → `TIMED_MOVE`: `timedMove(fwd, strafe, turn, seconds)` called (Auto).
- `TIMED_MOVE` → `IDLE`: timer ≥ seconds. Exposes `isBusy()` for the auto sequencer.
- any → `IDLE`: `stop()`.

**Voltage compensation (TIMED_MOVE only):** commanded power × (`DRIVE_NOMINAL_VOLTAGE` 12.0 V ÷ measured battery voltage), clamped to ±1.0. Battery voltage is sampled at the start of each timed move (not every loop, so it doesn't fight load-sag mid-move). Toggle `DRIVE_VOLTAGE_COMP_ENABLED` in `RoboConstants`.

**Dependencies:** none.

---

## Intake

**Motors:** `intake` — 5203 1150 RPM, EH M0, default direction FORWARD (verify: positive = pulls balls in). Plain power, no velocity control, no encoder use. Zero-power FLOAT.

**Sensors:** none.

**States**

| State | Power |
|---|---|
| `OFF` | 0 |
| `INTAKING` | `INTAKE_POWER` (default 1.0) |
| `REVERSING` | `INTAKE_REVERSE_POWER` (default −1.0) |

Transitions: set directly by the OpMode via `setState(...)` each loop (see Coordination below).

**Dependencies:** none inside the subsystem; the OpMode runs it during intake, shooting, and match-loader-UP.

---

## Indexer

Dumper-style: holds **4 balls**, takes in everything, shoots everything at once.

**Motors:** `indexer` — 5203 1150 RPM, EH M1, default direction FORWARD (verify: positive = moves balls toward the shooter). Plain power, no velocity control. Zero-power BRAKE (so balls don't drift into the stopper).

**Sensors:** none (no ball counting).

**States**

| State | Power |
|---|---|
| `OFF` | 0 |
| `INTAKING` | `INDEXER_INTAKE_POWER` (default 0.6 — tune) |
| `FEEDING` | `INDEXER_FEED_POWER` (default 1.0 — "runs fast while shooting") |
| `REVERSING` | `INDEXER_REVERSE_POWER` (default −1.0) |

Transitions: set by the OpMode each loop (see Coordination).

**Dependencies:** none inside the subsystem.

---

## Stopper

**Servos:** `stopper` — goBILDA positional, CH S0.

**Sensors:** none.

**States**

| State | Servo position |
|---|---|
| `CLOSED` | `STOPPER_CLOSED_POS` (default 0.0 — tune) |
| `OPEN` | `STOPPER_OPEN_POS` (default 0.5 — tune) |

- Initial state on init: `CLOSED`.
- Transitions set by the OpMode: `open()` / `close()`.

**Dependencies:** must only open when Flywheel reports `AT_SPEED` (enforced by the shoot logic in the OpMode).

---

## Flywheel

**Motors:** 2× goBILDA 5203 bare motor (6000 RPM, 28 ticks/rev at motor), driving one flywheel together:
- `flywheelA`: EH M2, default direction FORWARD.
- `flywheelB`: EH M3, default direction REVERSE. This assumes the two motors face each other on one shaft; verify that both push the wheel the same way.

Both motors get the same PIDF power. RPM is the average of the two built-in encoders. `RUN_WITHOUT_ENCODER` (we run our own loop), zero-power FLOAT.

- Gear ratio unknown → `FLYWHEEL_GEAR_RATIO = 1.0` placeholder, defined as motor revs per wheel rev (wheel RPM = motor RPM ÷ ratio).
- Target RPM unknown → `FLYWHEEL_TARGET_RPM` placeholder 3000 (wheel RPM), tune.

**Control:** DECODE-style PIDF on measured RPM: `pwr = kP·err + kI·∫err + kD·d(err) + kF·target`, clamped to [0, 1] (no reverse). Gains `FLYWHEEL_kP/kI/kD/kF` in `RoboConstants`, tunable via Panels. Integral reset when going to `OFF`.

**Sensors:** motor encoder (velocity).

**States**

| State | Behavior |
|---|---|
| `OFF` | Power 0, integral reset. Initial state. |
| `SPINNING_UP` | PIDF toward `FLYWHEEL_TARGET_RPM`; RPM outside tolerance. |
| `AT_SPEED` | PIDF continues; \|target − RPM\| ≤ `FLYWHEEL_RPM_TOLERANCE` (default 50). |

Transitions:
- `OFF` → `SPINNING_UP`: `spinUp()`.
- `SPINNING_UP` ↔ `AT_SPEED`: evaluated every `periodic()` from the tolerance check.
- any → `OFF`: `stop()`.
- Exposes `isAtSpeed()` and `getRpm()`.

**Dependencies:** none inside; Stopper/Indexer gate on `isAtSpeed()` in the OpMode.

---

## Match Loader

Lifts the plate so the robot can collect balls out of a FLOWER.

**Servos:** `matchLoader` — goBILDA positional, CH S1.

**Sensors:** none.

**States**

| State | Servo position |
|---|---|
| `DOWN` | `MATCH_LOADER_DOWN_POS` (default 0.0 — tune) |
| `UP` | `MATCH_LOADER_UP_POS` (default 0.5 — tune) |

- Initial state on init: `DOWN`.
- `toggle()`, `up()`, `down()`.

**Dependencies:** while `UP`, intake + indexer run `INTAKING` automatically (enforced in the OpMode).

---

## Coordination (TeleOp logic, lives in the OpMode)

Evaluated every loop, highest priority first:

1. **Reverse held** → Intake `REVERSING`, Indexer `REVERSING`, Stopper `CLOSED`. Flywheel follows rule 4.
2. **Shoot held** →
   - Flywheel `spinUp()` (automatically, even if the spin button isn't held).
   - Until the flywheel first reports `AT_SPEED`: Stopper `CLOSED`, Indexer `OFF`, Intake `OFF`.
   - Once `AT_SPEED`: Stopper `OPEN`, Indexer `FEEDING`, Intake `INTAKING`. **Latched** for the rest of this hold — a momentary RPM dip as balls go through doesn't re-close the stopper (dumper empties in one go).
   - Release → Stopper `CLOSED`, latch cleared.
3. **Intake held OR Match Loader is `UP`** → Intake `INTAKING`, Indexer `INTAKING`, Stopper `CLOSED`.
4. **Flywheel:** `spinUp()` while Spin held or Shoot held; otherwise `stop()`.
5. Otherwise → Intake `OFF`, Indexer `OFF`, Stopper `CLOSED`.

Integration decisions:
- Pressing Reverse **clears** the shoot latch. So if Shoot is still held after Reverse is released, the stopper waits for `AT_SPEED` again.
- Stopper and Match Loader servos are commanded to CLOSED / DOWN in their constructors, so they hold position from init.
- Motor directions are `final`. They're applied only at init, so flip them in code rather than in Panels.

---

## OpModes

### TeleOp — `JoJaTeleOp` (`@TeleOp(name = "JoJa Tele", group = "Competition")`)
- One driver (gamepad1 only). Wires all six subsystems; bindings per `control-mapping.md`; Coordination rules above.
- Telemetry: each subsystem's block + loop time.

### Autonomous — time + voltage based (30 s)

Both autos share a small timed-step sequencer (list of steps, each runs until done, then the next starts). Step types: `drive(fwd, strafe, turn, seconds)`, `spinUp()`, `shoot(seconds)` (waits for `AT_SPEED`, then opens stopper + feeds), `wait(seconds)`, `matchLoader(UP/DOWN)`.

**Alliance select during init:** gamepad1 X = Blue, B = Red (shown on telemetry; default Blue). Red mirrors every strafe and turn sign.

All durations/powers are **placeholders to tune on the field**, kept as constants at the top of each OpMode (Panels-tunable).

Sequencer: `opmodes/JoJaAutoBase.java`. Its `shoot` step runs the intake as well as the indexer, matching TeleOp. If the flywheel hasn't reached speed within `SHOOT_SPINUP_TIMEOUT_S` (2.5 s), it shoots anyway, so a bad RPM target can't stall auto.

1. **`JoJaAutoNear`** (`@Autonomous(name = "JoJa Auto Near HIVE")`) — starts near the HIVE with 4 preloads.
   1. Spin up flywheel.
   2. Drive backward 0.5 power for 0.8 s (into shooting range).
   3. Shoot for 2.0 s (dump all 4).
   4. Stop flywheel.
   5. Strafe out 0.5 power for 1.2 s (park/leave start).
2. **`JoJaAutoFar`** (`@Autonomous(name = "JoJa Auto Far")`) — starts at the far position with 4 preloads.
   1. Spin up flywheel.
   2. Drive forward 0.6 power for 1.8 s (toward the HIVE).
   3. Turn 0.4 power for 0.5 s (face the HIVE).
   4. Shoot for 2.0 s.
   5. Stop flywheel.
   6. Drive forward 0.5 power for 0.8 s (park).

---

## Vendor Dependencies

| Library | Version | Use |
|---|---|---|
| FTC SDK (official base) | 12.0.0 | — |
| FTCLib core (`org.ftclib.ftclib:core`) | 2.1.1 | `SubsystemBase`, `GamepadEx` |
| Panels (`com.bylazar:fullpanels`, repo `https://mymaven.bylazar.com/releases`) | 1.0.9 | Live tuning / telemetry |

- **Dropped from the reference:** Pedro Pathing, Pedro telemetry, FTCLib vision.
- **Risk to verify on first build:** FTCLib 2.1.1 and Panels 1.0.9 were used with SDK 11.0 in DECODE; SDK 12.0 compatibility hasn't been confirmed yet.
