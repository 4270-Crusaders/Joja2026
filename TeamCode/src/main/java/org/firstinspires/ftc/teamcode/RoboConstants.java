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
    // Drive voltage compensation (2026-11-09: now TeleOp too, every loop). Same stick = same speed at any battery.
    // 12.5 = roughly a fresh battery under load, so the robot feels like it always did on a fresh battery.
    public static double  DRIVE_NOMINAL_VOLTAGE      = 12.5;
    public static boolean DRIVE_VOLTAGE_COMP_ENABLED = true;
    // Load shedding (2026-11-05): while SHOOTING and a flywheel side is recovering, drive power is scaled down to
    // this at the worst dip (FLYWHEEL_FEED_MIN_RPM). The drivetrain is the biggest current draw on the robot.
    // 1.0 = off (driver always gets full power). Try 0.7 if shots dip while driving and shooting at once.
    public static double DRIVE_SHOOT_MIN_SCALE = 1.0;

    // ===== Intake =====
    // Expansion Hub motor port 3, goBILDA 5203 1150 RPM
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE; // flipped 2026-10-24: ran backward
    public static double INTAKE_POWER         = 1.0;
    public static double INTAKE_REVERSE_POWER = -1.0;
    // JoJa Tele: intake power while shooting (flywheel on + stopper open) without the left trigger.
    // Keeps balls moving up to the indexer. Used for ALL shooting, left trigger held or not. (2026-11-04: 0.5 -> 0.7)
    public static double INTAKE_SHOOT_POWER   = 0.7;
    // Proportional feed (2026-11-05): while a flywheel side recovers from a ball, the intake backs off toward this
    // (more battery for the flywheel). Feed scale 1 = INTAKE_SHOOT_POWER, 0 = this. See FLYWHEEL_FEED_FULL/MIN_RPM.
    public static double INTAKE_SHOOT_MIN_POWER = 0.4;

    // ===== Indexer =====
    // Control Hub motor port 3, goBILDA 5203 3.7:1 (~1620 RPM). Swapped from 1150 RPM 2026-11-04
    public static final String INDEXER_MOTOR_NAME = "indexer";
    public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.REVERSE; // flipped 2026-10-24: ran backward
    // 2026-11-04: both raised to full speed (were intake 0.6, feed 0.8). 1.0 is the motor's max.
    public static double INDEXER_INTAKE_POWER  = 0.5;   // 2026-11-04: halved (was 1.0) at team request

    public static double INDEXER_FEED_POWER = 0.9;   // 2026-11-11: 90% (was 1.0) at team request
    // 2026-11-04 (3.7:1 motor): the feed gate used to STOP the indexer between balls, so it restarted from a dead
    // stop every ball. Each restart pulls a big current spike (brownout) and wastes time (slow balls per second).
    // Now, while the flywheel recovers, the indexer just slows to this power instead of stopping.
    // Balls/sec still slow? Raise it. Flywheel dips deeper? Lower it. 0 = stop like before.
    public static double INDEXER_FEED_SLOW_POWER = 0.35;
    // How fast the indexer power may RISE, in power per second (5 = 0 to full in 0.2 s). Cuts the start-up current
    // spike that browns out the hub. Slowing down / stopping is always instant. Still browning out? Lower it.
    public static double INDEXER_RAMP_PER_SEC = 20.0; // 2026-11-05: 5 -> 20 (0 to full in 0.05 s) for max feed speed. Brownouts? Back to 5.
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
    public static double FLYWHEEL_TARGET_RPM    = 3255; // wheel RPM (set by team 2026-10-24)
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
    // ===== Auto shoot (JoJa Tele, 2026-11-09, subsystems/AutoShoot) =====
    // Hold RT: once BOTH sides are within READY_RPM of target for READY_S, the stopper opens by itself.
    // Each ball shows up as a dip of more than HIT_RPM. The stopper closes by itself once no new dip for EMPTY_S
    // (balls gone), or no ball at all within NO_BALL_S, or after MAX_S. Once per RT press. R1 still works (cancels auto).
    public static boolean AUTO_SHOOT_ENABLED  = true;
    public static double AUTO_SHOOT_READY_RPM = 50.0;  // opens too late / never? raise it. First shot weak? lower it.
    public static double AUTO_SHOOT_READY_S   = 0.15;  // must stay at speed this long (not just touch the target)
    public static double AUTO_SHOOT_HIT_RPM   = 60.0;  // dip size that counts as a ball. Missed balls? lower it.
    public static double AUTO_SHOOT_EMPTY_S   = 0.6;   // closes too early mid-volley? raise it. Stays open too long? lower it.
    public static double AUTO_SHOOT_NO_BALL_S = 1.0;
    public static double AUTO_SHOOT_MAX_S     = 4.0;
    // MASTER SWITCH (2026-11-05): false = NO feed gate. Indexer + intake feed at full speed the whole time the stopper
    // is open, no waiting for spin-up, no slowing during dips (like before the gate existed). Fastest balls/sec.
    // true = everything below (spin-up wait + proportional feed). Live-tunable in Panels.
    public static boolean FLYWHEEL_FEED_GATE_ENABLED = false;
    // Feed gate (2026-11-03): a POLLEN hitting both wheels took them to ~2800, and the next ball came in before they
    // recovered. Now the indexer PAUSES while either side is more than FEED_PAUSE below target, and feeds again once
    // both are back within FEED_RESUME. Each ball leaves at ~target speed, and the dips can't stack up.
    // Shooting too slow (long pauses)? Raise FEED_PAUSE. Still dipping deep? Lower it.
    public static double FLYWHEEL_FEED_PAUSE_RPM  = 100.0;
    public static double FLYWHEEL_FEED_RESUME_RPM = 40.0;
    // 2026-11-04: the pause could lock up: a POLLEN already touching the wheel was held there by the braked indexer,
    // so the wheel never sped back up and the indexer never restarted (flywheel stalled). Now a pause lasts at most
    // this long; then the indexer feeds anyway (pushes the ball through) until both sides are back near target.
    public static double FLYWHEEL_FEED_MAX_PAUSE_S = 0.25;
    // Proportional feed (2026-11-05): instead of full / slow, the indexer + intake slow down IN PROPORTION to the dip:
    //   worst side within FEED_FULL_RPM of target -> full feed (INDEXER_FEED_POWER / INTAKE_SHOOT_POWER)
    //   worst side FEED_MIN_RPM or more below     -> minimum feed (INDEXER_FEED_SLOW_POWER / INTAKE_SHOOT_MIN_POWER)
    //   in between                                -> in between
    // Faster balls/sec: raise FEED_FULL (feeds at full through small dips). Steadier RPM: lower it.
    public static double FLYWHEEL_FEED_FULL_RPM = 50.0;
    public static double FLYWHEEL_FEED_MIN_RPM  = 200.0;
    // The two flywheel motors are NOT mechanically connected, so each side has its own loop and gains.
    // 2026-11-05 rework (see Flywheel.java): rotational feedforward kS + kV*target does almost all the work,
    // then feedback: kP*error + kI*integral + kD*(-accel) + kA*(slowing rate). Error in wheel RPM, accel in RPM/s.
    // kD now acts on the measured speed (not the error), so it damps without kicking. 0 = off.
    // 2026-10-29: old kP/kI (0.001 / 0.0005) oscillated +/-300 RPM. kV does most of the work (~0.58 power at 3400),
    // kP only fixes what kV misses, kI slowly removes the leftover (friction) error.
    // If it still oscillates: lower kP. If it settles below target too slowly: raise kI a little.
    // 2026-11-02: kP 0.00015 / kI 0.0003 were too weak to hold the speed (it sagged ~500 RPM). Doubled both.
    // Shaking fast around target (+/-100 every few tenths of a second)? Lower kP. Slow drift below target? Raise kI.
    public static double FLYWHEEL_LEFT_kP  = 0.0001125;  // SysId 2026-11-05
    public static double FLYWHEEL_LEFT_kI  = 0.0010682;  // SysId 2026-11-05
    public static double FLYWHEEL_LEFT_kD  = 0.0;  // SysId 2026-11-05
    public static double FLYWHEEL_LEFT_kV  = 0.0001909;  // SysId 2026-11-05
    public static double FLYWHEEL_LEFT_kS  = 0.1911;  // SysId 2026-11-05
    public static double FLYWHEEL_LEFT_kA  = 0.0000201;  // SysId 2026-11-05

    public static double FLYWHEEL_RIGHT_kP = 0.0000989;  // SysId 2026-11-05
    public static double FLYWHEEL_RIGHT_kI = 0.0009787;  // SysId 2026-11-05
    public static double FLYWHEEL_RIGHT_kD = 0.0;  // SysId 2026-11-05
    public static double FLYWHEEL_RIGHT_kV = 0.0001782;  // SysId 2026-11-05
    public static double FLYWHEEL_RIGHT_kS = 0.1427;  // SysId 2026-11-05
    public static double FLYWHEEL_RIGHT_kA = 0.0000180;  // SysId 2026-11-05

    // Smoothing for the measured acceleration, in SECONDS (2026-11-05: was a per-loop 0.3, which changed with the
    // loop speed). Encoder velocity is noisy; this filters out the noise. Bigger = smoother but reacts later.
    // kA / kD power jittery at steady speed? Raise it (0.05). Want faster reaction to hits? Lower it (0.02).
    public static double FLYWHEEL_ACCEL_FILTER_S = 0.03;

    // ===== Match Loader =====
    // Expansion Hub servo port 0, goBILDA positional servo
    public static final String MATCH_LOADER_SERVO_NAME = "matchLoader";
    public static double MATCH_LOADER_DOWN_POS = 0.42; // deployed down (JoJa Match Loader Test, 2026-11-09)
    public static double MATCH_LOADER_UP_POS   = 0.742; // closed / up (JoJa Match Loader Test, 2026-11-09)
    public static double MATCH_LOADER_MIDDLE_POS = 0.524; // pollen-only gap, no nectar (JoJa Match Loader Test, 2026-11-09)
    // Trim added to ALL three positions (fixes a servo horn screwed on at the wrong angle).
    // Adjust live in JoJa Tele with the D-pad, then copy the value shown on the Driver Hub here.
    public static double MATCH_LOADER_OFFSET = 0.0;

    // ===== Odometry (goBILDA Pinpoint + 2 pods) =====
    // Control Hub I2C bus 1. Driver Hub config: "goBILDA Pinpoint Odometry Computer" named "pinpoint"
    public static final String PINPOINT_NAME = "pinpoint";
    public static boolean PINPOINT_4_BAR_POD = true;  // true = 4-Bar Odometry Pod, false = Swingarm Pod (Pedro AutoTune 2026-11-10)
    // ONE place for the Pinpoint settings: Pedro (pedro/Constants), JoJa Tele and JoJa Odometry Test all read these.
    // Tune with Pedro AutoTune > Pinpoint Procedure (see pedro/Tuning.java), then copy its Java tab values HERE:
    //   xPodOffset -> PINPOINT_X_OFFSET_IN, yPodOffset -> PINPOINT_Y_OFFSET_IN,
    //   xPodDirection / yPodDirection REVERSED -> PINPOINT_X_REVERSED / PINPOINT_Y_REVERSED = true, podType -> PINPOINT_4_BAR_POD
    // Offsets from the robot center, inches (Pedro AutoTune > Pinpoint Procedure, 2026-11-10):
    // X offset = how far LEFT (+) / right (-) the FORWARD pod (X port) is. Y offset = how far FORWARD (+) / back (-) the STRAFE pod (Y port) is.
    // Average of 2 counterclockwise runs: X 1.811 / 1.522, Y -5.371 / -3.218 (Y runs 2.2 in apart: redo if Loc Test 3 drifts)
    public static double PINPOINT_X_OFFSET_IN = 1.666;
    public static double PINPOINT_Y_OFFSET_IN = -4.294; // the strafe pod is ~4.3 in BEHIND the center
    // Push robot forward: X should go UP. Push left: Y should go UP. If not, flip it (JoJa Odometry Test D-pad L/R).
    // Pedro AutoTune 2026-11-10: xPodDirection FORWARD, yPodDirection REVERSED. 2026-11-11: X was flipped to true
    // by hand and every Loc Test / R1 broke (forward read as backward). Only change these after AutoTune says so.
    public static boolean PINPOINT_X_REVERSED = false;
    public static boolean PINPOINT_Y_REVERSED = true;

    // ===== JoJa Tele A button: "the robot is HERE" (2026-11-11) =====
    // A sets x = A_X, y = A_Y AND heading = A_HEADING_DEG (also the pose at Init).
    // Heading = the way the robot's FRONT faces on the Pedro field:
    //   0 = right (+X)   90 = up the field (+Y)   180 = left (-X)   -90 = down (-Y)
    // Wrong heading = the whole map is turned: driving into the field gives NEGATIVE x / y. Never fix that by
    // reversing a pod (that breaks Pedro): fix this heading.
    public static double A_X = 0.0;
    public static double A_Y = 0.0;
    public static double A_HEADING_DEG = 0.0;

    // ===== JoJa Tele R1: drive to the shooting spot with Pedro (2026-11-11) =====
    // Pedro field: inches, (0,0) = bottom-left corner, heading 0 = facing +X, counterclockwise +.
    // Which spot R1 goes to depends on where the robot is when you press it (2026-11-11):
    //   robot on spot 1's side of TRACK_SPOT1_EDGE_X -> spot 1
    //   robot on spot 2's side of TRACK_SPOT2_EDGE_X -> spot 2      in between -> nothing
    // "Side" comes from where each spot's own x is, so negative or positive numbers both work.
    // Now: spot 1 x = -7 is above -50, so x > -50 (0 to -50) = spot 1. Spot 2 x = -115 is below -80, so x < -80 = spot 2.
    public static double TRACK_SPOT1_EDGE_X = -50.0;
    public static double TRACK_SPOT2_EDGE_X = -80.0;
    // Spot 1
    public static double TRACK_TARGET_X = -8.00;
    public static double TRACK_TARGET_Y = 53.15;
    public static double TRACK_TARGET_HEADING_DEG = 0;
    // Spot 2
    public static double TRACK_TARGET2_X = -116.00;
    public static double TRACK_TARGET2_Y = 54.15;
    public static double TRACK_TARGET2_HEADING_DEG = 180;
    // Moving a stick more than this while it drives = you take over (cancel).
    public static double TRACK_CANCEL_STICK = 0.2;

    // ====== Battery / voltage compensation (2026-11-09, utils/Battery) ======
    // Intake + indexer + drive powers are scaled by NOMINAL / battery, so they run the same speed as the battery drains.
    // (The flywheel always compensates, to its own FLYWHEEL_NOMINAL_VOLTAGE: its SysId gains depend on it.)
    // Can't create volts the battery doesn't have: at power 1.0 there's nothing left to add.
    public static boolean BATTERY_COMP_ENABLED   = true;
    public static double  BATTERY_NOMINAL_VOLTAGE = 12.5; // the battery level the powers were tuned at
    public static double  BATTERY_LOW_WARNING_V   = 11.8; // Driver Hub warning: swap the battery soon

    // ====== Utilities ======
    public static final double LOOP_HZ = 50.0;
    // Fast loop (2026-11-05, utils/LoopTimer): every loop takes at least LOOP_MIN_PERIOD_MS (steady timing for
    // the flywheel math); hardware reads/writes decide the rest, aim is 5-10 ms. Telemetry (slow: text + sending)
    // only every TELEMETRY_PERIOD_MS. Watch "Loop avg / max ms" on the Driver Hub.
    public static double LOOP_MIN_PERIOD_MS  = 5.0;
    public static double TELEMETRY_PERIOD_MS = 50.0;
    // Motor writes are skipped when the new power is within this of the last one sent (each write is a hub
    // command, ~1-3 ms). Small enough that nothing can feel it.
    public static double MOTOR_WRITE_DEADBAND = 0.005;
    // Current monitor (JoJa Tele): shows amps per motor ("Amps/...") + total. Reads ONE motor per loop
    // (each read is a hub command, ~1-2 ms), so turn it on only while analyzing. Live-tunable in Panels.
    public static boolean CURRENT_MONITOR = false;
    public static final double dt(ElapsedTime t) { return t.seconds(); }
}
