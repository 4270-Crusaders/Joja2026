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

    // ===== Indexer =====
    // Control Hub motor port 3, goBILDA 5203 1150 RPM
    public static final String INDEXER_MOTOR_NAME = "indexer";
    public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.REVERSE; // flipped 2026-10-24: ran backward
    public static double INDEXER_INTAKE_POWER  = 0.6;   // TUNE

    public static double INDEXER_FEED_POWER = 0.8;
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
    public static double FLYWHEEL_TARGET_RPM    = 3400; // wheel RPM (set by team 2026-10-24)
    public static double FLYWHEEL_RPM_TOLERANCE = 500.0; // tuned 2026-10-24:
    // wide so the stopper opens reliably
    public static double FLYWHEEL_I_ZONE_RPM = 300.0;  // integral only builds within this many RPM of target
    // The two flywheel motors are NOT mechanically connected, so each side has its own PIDF.
    // power = kF*target + kP*error + kI*integral + kD*d(error); error in wheel RPM. Tune in JoJa Flywheel Tuner.
    public static double FLYWHEEL_LEFT_kP  = 0.001;  // tuned 2026-10-24
    public static double FLYWHEEL_LEFT_kI  = 0.001;  // tuned 2026-10-24
    public static double FLYWHEEL_LEFT_kD  = 0.0;  // tuned 2026-10-24
    public static double FLYWHEEL_LEFT_kF  = 0.00017;  // tuned 2026-10-24
    public static double FLYWHEEL_RIGHT_kP = 0.0005;  // tuned 2026-10-24
    public static double FLYWHEEL_RIGHT_kI = 0.0005;  // tuned 2026-10-24
    public static double FLYWHEEL_RIGHT_kD = 0.0;  // tuned 2026-10-24
    public static double FLYWHEEL_RIGHT_kF = 0.00017;  // tuned 2026-10-24

    // ===== Match Loader =====
    // Expansion Hub servo port 0, goBILDA positional servo
    public static final String MATCH_LOADER_SERVO_NAME = "matchLoader";
    public static double MATCH_LOADER_DOWN_POS = 0.3; // deployed down (set by team 2026-10-24)
    public static double MATCH_LOADER_UP_POS   = 0.66; // closed / up (set by team 2026-10-24)
    public static double MATCH_LOADER_MIDDLE_POS = 0.465; // pollen-only gap, no nectar. GUESS (halfway): TUNE on the robot
    // Trim added to ALL three positions (fixes a servo horn screwed on at the wrong angle).
    // Adjust live in JoJa Tele with the D-pad, then copy the value shown on the Driver Hub here.
    public static double MATCH_LOADER_OFFSET = 0.0;

    // ====== Utilities ======
    public static final double LOOP_HZ = 50.0;
    public static final double dt(ElapsedTime t) { return t.seconds(); }
}
