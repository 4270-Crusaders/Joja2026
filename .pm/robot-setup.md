# JoJa First-Time Robot Setup

Do these in order. Each step depends on the one before it.

## 1. Driver Station configuration

On the Driver Station, go to **⋮ → Configure Robot → New**. Scan, then name the devices **exactly** as below. Names are case-sensitive.

**Control Hub**

| Section | Port | Device type | Name |
|---|---|---|---|
| Motors | 0 | GoBILDA 5202/3/4 series | `leftBack` |
| Motors | 1 | GoBILDA 5202/3/4 series | `leftFront` |
| Motors | 2 | GoBILDA 5202/3/4 series | `flywheelLeft` |
| Motors | 3 | GoBILDA 5202/3/4 series | `indexer` |
| Servos | 0 | Servo | `stopper` |
| I2C Bus 0 | 0 | (built-in IMU, usually already there) | `imu` |

**Expansion Hub**

| Section | Port | Device type | Name |
|---|---|---|---|
| Motors | 0 | GoBILDA 5202/3/4 series | `rightBack` |
| Motors | 1 | GoBILDA 5202/3/4 series | `rightFront` |
| Motors | 2 | GoBILDA 5202/3/4 series | `flywheelRight` |
| Motors | 3 | GoBILDA 5202/3/4 series | `intake` |
| Servos | 0 | Servo | `matchLoader` |

Save it, then **activate** that configuration.

If you wire things differently, change the names or ports in this table *or* in `RoboConstants.java` so the two match.

## 2. Hardware test

Run **JoJa Hardware Test** (TeleOp, group "Test") with the robot **on a stand**.

1. **Directions:** for each motor, hold **A** and check that it does what the telemetry line "A should …" says. If one goes the wrong way, flip its `*_DIRECTION` in `RoboConstants.java`.
2. **Flywheel left vs right:** run each one alone. **Both must spin the wheel the same way.** If they fight each other, stop right away and flip `FLYWHEEL_RIGHT_DIRECTION`.
3. **Encoders:** when a motor runs forward, its encoder count should go **up**. If a drive motor's count goes down, flip its encoder direction in `pedroPathing/Constants.java`.
4. **Servo positions:** select `stopper`, nudge it to the closed position, and copy the number into `STOPPER_CLOSED_POS`. Repeat for open, and for `matchLoader` down and up.
5. **IMU:** turn the robot left by hand. The yaw should go **up**. If it goes down or barely changes, fix the IMU orientation (logo and USB direction) in `pedroPathing/Constants.java` and in this test OpMode.

## 3. TeleOp check

Run **JoJa Tele** on the floor.
- **Driving:** sticks forward, strafe right and turn right should all go the right way.
- **Intake:** hold left trigger. Balls should come in and the stopper should stay closed.
- **Shooting:** hold right trigger. The flywheel spins up, then the stopper opens and the robot dumps.
- **Flywheel tuning:** tune `FLYWHEEL_TARGET_RPM`, `kF` and `kP` in Panels until the `Flywheel/RPM` telemetry reaches the target quickly without overshooting.

## 4. Pedro tuning

1. Measure and set these in `pedroPathing/Constants.java`:
   - robot mass (kg)
   - wheel diameter (96 or 104 mm)
   - robot length and width
2. Run **Tuning** (TeleOp, group "Pedro Pathing") in this order:
   1. LocalizationTest
   2. ForwardTuner
   3. LateralTuner
   4. ForwardVelocityTuner
   5. LateralVelocityTuner
   6. the deceleration tuners
   7. the PIDF tuners
3. Copy each result into `Constants.java`.

## 5. Autos

Measure JoJa's START, SHOOT and PARK positions on the field, in Pedro coordinates (inches, 0–144). Set them at the top of `JoJaAutoNear` and `JoJaAutoFar`. Run each auto as Blue, then as Red.
