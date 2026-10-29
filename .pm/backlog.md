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
- `pedroPathing/` (Constants + Tuning) and `myLocalizer/` (DECODE's drive-encoder + IMU localizer) ported from the reference. Pedro is used **in autonomous only**; TeleOp drives through the Drivetrain subsystem.
- Every subsystem extends FTCLib `SubsystemBase`, constructor `(HardwareMap hMap, String... deviceNames, Telemetry telemetry)`, logs `RobotLog.i("[Name] Initialized.")`, and does its hardware writes + its own telemetry block in `periodic()`.
- All hardware map names and tuning values live in `RoboConstants`. Values that need tuning are `public static` (non-final) so Panels can edit them live.
- OpModes are iterative `OpMode`; gamepad wrapped in FTCLib `GamepadEx`. OpModes call every subsystem's `periodic()` once per loop.
- Cross-subsystem coordination (who runs the intake/indexer when) lives in the OpMode, not inside subsystems. Subsystems expose states; the OpMode decides.

## Hardware Summary

All ports below are the team's actual wiring (confirmed 2026-10-23). Names are PM defaults — **verify on the robot and match the Driver Station configuration to these names.** Motor directions are defaults, also to verify.

| Device | Type | HW map name | Hub | Port |
|---|---|---|---|---|
| Left front drive | goBILDA 5203 435 RPM | `leftFront` | Control Hub | Motor 1 |
| Left back drive | goBILDA 5203 435 RPM | `leftBack` | Control Hub | Motor 0 |
| Right front drive | goBILDA 5203 435 RPM | `rightFront` | Expansion Hub | Motor 1 |
| Right back drive | goBILDA 5203 435 RPM | `rightBack` | Expansion Hub | Motor 0 |
| Intake | goBILDA 5203 1150 RPM | `intake` | Expansion Hub | Motor 3 |
| Indexer | goBILDA 5203 1150 RPM | `indexer` | Control Hub | Motor 3 |
| Flywheel left | goBILDA 5203 bare (6000 RPM, 28 TPR) | `flywheelLeft` | Control Hub | Motor 2 |
| Flywheel right | goBILDA 5203 bare (6000 RPM, 28 TPR) | `flywheelRight` | Expansion Hub | Motor 2 |
| Ball stopper | goBILDA positional servo | `stopper` | Control Hub | Servo 0 |
| Match loader | goBILDA positional servo | `matchLoader` | Expansion Hub | Servo 0 |
| Battery voltage | Built-in hub voltage sensor | (first `VoltageSensor` in hardwareMap) | — | — |

Control Hub built-in IMU (`imu`) is used by the Pedro localizer in auto. Drive motor encoders are read by the localizer in auto. No camera/Limelight, no other sensors.

No odometry pods (decided): localization stays on drive encoders + IMU.

---

## Drivetrain

**Motors**

| Name | Model | Hub / Port | Direction (default) | Encoder |
|---|---|---|---|---|
| `leftFront` | 5203 435 RPM | CH M1 | REVERSE | not used |
| `leftBack` | 5203 435 RPM | CH M0 | REVERSE | not used |
| `rightFront` | 5203 435 RPM | EH M1 | FORWARD | not used |
| `rightBack` | 5203 435 RPM | EH M0 | FORWARD | not used |

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

**Motors:** `intake` — 5203 1150 RPM, EH M3, default direction FORWARD (verify: positive = pulls balls in). Plain power, no velocity control, no encoder use. Zero-power FLOAT.

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

**Motors:** `indexer` — 5203 1150 RPM, CH M3, default direction FORWARD (verify: positive = moves balls toward the shooter). Plain power, no velocity control. Zero-power BRAKE (so balls don't drift into the stopper).

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
- `flywheelLeft`: CH M2, default direction FORWARD.
- `flywheelRight`: EH M2, default direction REVERSE. This assumes the two motors face each other on one shaft; verify that both push the wheel the same way.

Each motor runs its **own** PIDF loop to the same target RPM, using its own encoder, so a side with more friction gets more power. `AT_SPEED` requires **both** sides within tolerance. (Changed 2026-10-23 after the two sides measured very different speeds on the same power.) `RUN_WITHOUT_ENCODER` (we run our own loop), zero-power FLOAT.

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

**Servos:** `matchLoader` — goBILDA positional, EH S0.

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
3. **Intake held** → Intake `INTAKING`, Indexer `INTAKING`, Stopper `CLOSED`. (The match loader is a plain toggle and does not start the intake.)
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

### Autonomous (timed, no Pedro)

1. **`JoJaAutoShootAndBack`** (`@Autonomous(name = "JoJa Auto Shoot and Back")`). Steps, all at 0.5 power and tuned on the robot 2026-10-24:
   1. Strafe left 2.15 s.
   2. Drive forward 1 s.
   3. Flywheel on at **3400 RPM** (this auto only), then wait 2 s.
   4. Open the stopper and feed for 3 s. Then flywheel off and stopper closed.
   5. Drive forward 0.5 s.
   6. Strafe right 1.5 s.
   7. Drive backward 3.5 s.
   8. Strafe right 1 s.

   About 14.7 s in total.
2. **`JoJaAutoForward1s`** (`@Autonomous(name = "JoJa Auto Forward 1s")`): drives forward 1 s at 0.5 power, then stops.

Both use the Drivetrain subsystem's voltage-compensated `timedMove`. Times and powers are constants at the top of each file.

The Pedro-based autos (Near, Far, Forward + Right test, and `JoJaAutoBase`) were removed on 2026-10-24. Pedro (`pedroPathing/`, `myLocalizer/`, Tuning) is still in the project for later.

---

## Vendor Dependencies

| Library | Version | Use |
|---|---|---|
| FTC SDK (official base) | 12.0.0 | — |
| FTCLib core (`org.ftclib.ftclib:core`) | 2.1.1 | `SubsystemBase`, `GamepadEx` |
| Panels (`com.bylazar:fullpanels`, repo `https://mymaven.bylazar.com/releases`) | 1.0.9 | Live tuning / telemetry |
| Pedro Pathing (`com.pedropathing:ftc`) | 2.0.4 | Auto path following (same version as DECODE; the ported localizer and Tuning code targets it. 2.1.2 exists but is untested here) |
| Pedro telemetry (`com.pedropathing:telemetry`) | 1.0.0 | Tuning OpModes |

- **Dropped from the reference:** FTCLib vision.
- **Risk to verify on first build:** FTCLib 2.1.1, Panels 1.0.9 and Pedro 2.0.4 were used with SDK 11.0 in DECODE; SDK 12.0 compatibility hasn't been confirmed yet.
