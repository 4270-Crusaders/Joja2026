package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Non-final statics are live-tunable from Panels. Hardware names and motor
// directions are final: they're only applied when an OpMode initializes.
// All hardware names, ports, and directions are PM defaults: VERIFY ON ROBOT.
@Configurable
public final class RoboConstants {
    // ====== Drivetrain ======
    // goBILDA 5203 435 RPM. Left side on the Control Hub, right side on the Expansion Hub
    public static final String DRIVE_LEFT_FRONT_NAME  = "leftFront";   // CH M1
    public static final String DRIVE_LEFT_BACK_NAME   = "leftBack";    // CH M0
    public static final String DRIVE_RIGHT_FRONT_NAME = "rightFront";  // EH M1
    public static final String DRIVE_RIGHT_BACK_NAME  = "rightBack";   // EH M0
    public static final DcMotorSimple.Direction DRIVE_LEFT_FRONT_DIRECTION  = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction DRIVE_LEFT_BACK_DIRECTION   = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction DRIVE_RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction DRIVE_RIGHT_BACK_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    public static double  DRIVE_NOMINAL_VOLTAGE      = 12.0;
    public static boolean DRIVE_VOLTAGE_COMP_ENABLED = true;

    // ===== Intake =====
    // Expansion Hub motor port 3, goBILDA 5203 1150 RPM
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE; // flipped 2026-10-24: ran backward
    public static double INTAKE_POWER         = 1.0;
    public static double INTAKE_REVERSE_POWER = -1.0;
    // JoJa Tele: intake power while shooting (flywheel on + stopper open) without the left trigger.
    // Keeps balls moving up to the indexer. Used for ALL shooting, left trigger held or not. (2026-11-04: 0.5 -> 0.7)
    public static double INTAKE_SHOOT_POWER   = 0.7;

    // ===== Indexer =====
    // Control Hub motor port 3, goBILDA 5203 3.7:1 (~1620 RPM). Swapped from 1150 RPM 2026-11-04
    public static final String INDEXER_MOTOR_NAME = "indexer";
    public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.REVERSE; // flipped 2026-10-24: ran backward
    // 2026-11-04: both raised to full speed (were intake 0.6, feed 0.8). 1.0 is the motor's max.
    public static double INDEXER_INTAKE_POWER  = 0.5;   // 2026-11-04: halved (was 1.0) at team request

    public static double INDEXER_FEED_POWER = 1.0;
    // 2026-11-04 (3.7:1 motor): the feed gate used to STOP the indexer between balls, so it restarted from a dead
    // stop every ball. Each restart pulls a big current spike (brownout) and wastes time (slow balls per second).
    // Now, while the flywheel recovers, the indexer just slows to this power instead of stopping.
    // Balls/sec still slow? Raise it. Flywheel dips deeper? Lower it. 0 = stop like before.
    public static double INDEXER_FEED_SLOW_POWER = 0.35;
    // How fast the indexer power may RISE, in power per second (5 = 0 to full in 0.2 s). Cuts the start-up current
    // spike that browns out the hub. Slowing down / stopping is always instant. Still browning out? Lower it.
    public static double INDEXER_RAMP_PER_SEC = 5.0;
    public static double INDEXER_FEED_VOLTS    = 6.0;   // shooting: ~6 V to the motor, any battery level (raised from 5 V, 2026-10-24)
    public static double INDEXER_REVERSE_POWER = -1.0;

    // ===== Stopper =====
    // Control Hub servo port 0, goBILDA positional servo
    public static final String STOPPER_SERVO_NAME = "stopper";
    // Servo positions are absolute angles: 0.0 and 1.0 are the two ends of its 300 deg travel. Never negative.
    public static final Servo.Direction STOPPER_DIRECTION = Servo.Direction.FORWARD;
    // goBILDA speed servo: 0.0-1.0 spans ~270 deg on the SDK's default 600-2400us signal, so 90 deg = 0.333
    public static double STOPPER_CLOSED_POS = 0.25;  // intaking / holding balls (set by team 2026-10-24)
    public static double STOPPER_OPEN_POS   = 0.05;  // shooting (set by team 2026-10-24)

    // ====== Flywheel ======
    // 2x goBILDA 5203 bare motor (6000 RPM), same power to both
    public static final String FLYWHEEL_LEFT_MOTOR_NAME = "flywheelLeft";   // CH M2
    public static final String FLYWHEEL_RIGHT_MOTOR_NAME = "flywheelRight";  // EH M2
    // One motor on each side of the shooter, mirrored, so one is reversed to spin the wheel the same way. VERIFY
    public static final DcMotorSimple.Direction FLYWHEEL_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FLYWHEEL_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final int FLYWHEEL_TPR = 28;            // bare motor encoder, ticks per motor rev
    public static double FLYWHEEL_GEAR_RATIO    = 1.0;    // motor revs per wheel rev. JoJa: 1:1
    public static double FLYWHEEL_TARGET_RPM    = 3000; // wheel RPM (set by team 2026-10-24)
    // Panels graph zoom (JoJa Tele graph()): the y-axis shows TARGET +/- this many RPM. Anything outside is drawn
    // flat on the edge. Bigger = see more, smaller = see small wiggles. Live-tunable in Panels.
    public static double GRAPH_RPM_WINDOW = 75.0; // 2026-11-04: was 150, zoomed in more
    public static double FLYWHEEL_RPM_TOLERANCE = 500.0; // tuned 2026-10-24:
    // wide so the stopper opens reliably
    // 2026-10-29: zone was 300. A side that settled >300 below target never built any integral, so kI did nothing.
    public static double FLYWHEEL_I_ZONE_RPM = 1000.0; // integral only builds within this many RPM of target (held, not reset, outside)
    public static double FLYWHEEL_I_MAX_POWER = 0.4;   // the integral can add/remove at most this much power (anti-windup)
    public static double FLYWHEEL_NOMINAL_VOLTAGE = 12.0; // flywheel power is scaled by this / battery voltage (voltage compensation)
    // Fast spin-up (boost): full power until a side is within BOOST_BAND of target, then PIDF takes over.
    // Boost only happens once per spin-up. 2026-11-02: re-entering boost (500 RPM below) made a full-power /
    // too-little-power cycle that swung 2300-3100 RPM, so after spin-up the PIDF alone holds the speed.
    // Overshoots past target? Raise BOOST_BAND. Still slow near the end? Lower it.
    public static double FLYWHEEL_BOOST_BAND_RPM = 200.0;
    // Shot recovery (only while the stopper is open): full power whenever a side is more than this below target.
    // Small = fights every dip right away. If it overshoots above target while shooting, raise it a little.
    public static double FLYWHEEL_RECOVERY_BAND_RPM = 30.0;
    // Feed gate (2026-11-03): a POLLEN hitting both wheels took them to ~2800, and the next ball came in before they
    // recovered. Now the indexer PAUSES while either side is more than FEED_PAUSE below target, and feeds again once
    // both are back within FEED_RESUME. Each ball leaves at ~target speed, and the dips can't stack up.
    // Shooting too slow (long pauses)? Raise FEED_PAUSE. Still dipping deep? Lower it.
    public static double FLYWHEEL_FEED_PAUSE_RPM  = 100.0;
    public static double FLYWHEEL_FEED_RESUME_RPM = 40.0;
    // 2026-11-04: the pause could lock up: a POLLEN already touching the wheel was held there by the braked indexer,
    // so the wheel never sped back up and the indexer never restarted (flywheel stalled). Now a pause lasts at most
    // this long; then the indexer feeds anyway (pushes the ball through) until both sides are back near target.
    public static double FLYWHEEL_FEED_MAX_PAUSE_S = 0.25;    // The two flywheel motors are NOT mechanically connected, so each side has its own PIDF.
    // power = kS + kV*target + kA*(slowing rate) + kP*error + kI*integral + kD*d(error); error in wheel RPM. Tune in JoJa Flywheel Tuner.
    // 2026-10-29: old kP/kI (0.001 / 0.0005) oscillated +/-300 RPM. kV does most of the work (~0.58 power at 3400),
    // kP only fixes what kV misses, kI slowly removes the leftover (friction) error.
    // If it still oscillates: lower kP. If it settles below target too slowly: raise kI a little.
    // 2026-11-02: kP 0.00015 / kI 0.0003 were too weak to hold the speed (it sagged ~500 RPM). Doubled both.
    // Shaking fast around target (+/-100 every few tenths of a second)? Lower kP. Slow drift below target? Raise kI.
    public static double FLYWHEEL_LEFT_kP  = 0.0000801;
    public static double FLYWHEEL_LEFT_kI  = 0.0005736;
    public static double FLYWHEEL_LEFT_kD  = 0.0;  // tuned 2026-10-24
    public static double FLYWHEEL_LEFT_kV  = 0.0001606;  // tuned 2026-10-24
    public static double FLYWHEEL_RIGHT_kP = 0.0000767;
    public static double FLYWHEEL_RIGHT_kI = 0.0005493;
    public static double FLYWHEEL_RIGHT_kD = 0.0;  // tuned 2026-10-24
    public static double FLYWHEEL_RIGHT_kV = 0.0001538;  // tuned 2026-10-24
    // Static friction feedforward (power added whenever spinning). 0 = off. Get it from JoJa Flywheel SysId.
    public static double FLYWHEEL_LEFT_kS  = 0.1857;
    public static double FLYWHEEL_RIGHT_kS = 0.1596;
    // kA (2026-11-04): power per RPM/s of SLOWING DOWN. A POLLEN hitting the wheel makes it decelerate before the
    // RPM error is big; kA adds power right away in proportion to how fast it's slowing (only when slowing, never brakes).
    // Theory value = kV * tau (tau = JoJa Flywheel SysId "tau"). 0 = off. Starting guess assumes tau ~0.3 s.
    // Too jumpy / shaky at steady speed? Lower it. Dips still slow to react? Raise it.
    public static double FLYWHEEL_LEFT_kA  = 0.0000224;
    public static double FLYWHEEL_RIGHT_kA = 0.0000215;
    // Smoothing for the measured acceleration (0-1). Smaller = smoother but slower. Encoder velocity is noisy.
    public static double FLYWHEEL_ACCEL_SMOOTHING = 0.3;

    // ===== Match Loader =====
    // Expansion Hub servo port 0, goBILDA positional servo
    public static final String MATCH_LOADER_SERVO_NAME = "matchLoader";
    public static double MATCH_LOADER_DOWN_POS = 0.3; // deployed down (set by team 2026-10-24)
    public static double MATCH_LOADER_UP_POS   = 0.66; // closed / up (set by team 2026-10-24)
    public static double MATCH_LOADER_MIDDLE_POS = 0.465; // pollen-only gap, no nectar. GUESS (halfway): TUNE on the robot
    // Trim added to ALL three positions (fixes a servo horn screwed on at the wrong angle).
    // Adjust live in JoJa Tele with the D-pad, then copy the value shown on the Driver Hub here.
    public static double MATCH_LOADER_OFFSET = 0.0;

    // ===== Odometry (goBILDA Pinpoint + 2 pods) =====
    // Control Hub I2C bus 1. Driver Hub config: "goBILDA Pinpoint Odometry Computer" named "pinpoint"
    public static final String PINPOINT_NAME = "pinpoint";
    public static boolean PINPOINT_4_BAR_POD = true;  // true = 4-Bar Odometry Pod, false = Swingarm Pod. VERIFY
    // Offsets from the robot center, inches. PLACEHOLDERS (0) until measured:
    // X offset = how far LEFT (+) / right (-) the FORWARD pod (X port) is. Y offset = how far FORWARD (+) / back (-) the STRAFE pod (Y port) is.
    public static double PINPOINT_X_OFFSET_IN = 0.0;
    public static double PINPOINT_Y_OFFSET_IN = 0.0;
    // Push robot forward: X should go UP. Push left: Y should go UP. If not, flip it (JoJa Odometry Test D-pad L/R).
    public static boolean PINPOINT_X_REVERSED = false;
    public static boolean PINPOINT_Y_REVERSED = false;

    // ====== Utilities ======
    public static final double LOOP_HZ = 50.0;
    public static final double dt(ElapsedTime t) { return t.seconds(); }
}
