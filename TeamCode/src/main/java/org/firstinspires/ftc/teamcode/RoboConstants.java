package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// Non-final statics are live-tunable from Panels. Hardware names and motor
// directions are final: they're only applied when an OpMode initializes.
// All hardware names, ports, and directions are PM defaults: VERIFY ON ROBOT.
@Configurable
public final class RoboConstants {
    // ====== Drivetrain ======
    // Control Hub motor ports 0-3, goBILDA 5203 435 RPM
    public static final String DRIVE_LEFT_FRONT_NAME  = "leftFront";   // CH M0
    public static final String DRIVE_LEFT_BACK_NAME   = "leftBack";    // CH M1
    public static final String DRIVE_RIGHT_FRONT_NAME = "rightFront";  // CH M2
    public static final String DRIVE_RIGHT_BACK_NAME  = "rightBack";   // CH M3
    public static final DcMotorSimple.Direction DRIVE_LEFT_FRONT_DIRECTION  = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction DRIVE_LEFT_BACK_DIRECTION   = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction DRIVE_RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction DRIVE_RIGHT_BACK_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    public static double  DRIVE_NOMINAL_VOLTAGE      = 12.0;
    public static boolean DRIVE_VOLTAGE_COMP_ENABLED = true;

    // ===== Intake =====
    // Expansion Hub motor port 0, goBILDA 5203 1150 RPM
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static double INTAKE_POWER         = 1.0;
    public static double INTAKE_REVERSE_POWER = -1.0;

    // ===== Indexer =====
    // Expansion Hub motor port 1, goBILDA 5203 1150 RPM
    public static final String INDEXER_MOTOR_NAME = "indexer";
    public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static double INDEXER_INTAKE_POWER  = 0.6;   // TUNE
    public static double INDEXER_FEED_POWER    = 1.0;
    public static double INDEXER_REVERSE_POWER = -1.0;

    // ===== Stopper =====
    // Control Hub servo port 0, goBILDA positional servo
    public static final String STOPPER_SERVO_NAME = "stopper";
    public static double STOPPER_CLOSED_POS = 0.0;  // TUNE
    public static double STOPPER_OPEN_POS   = 0.5;  // TUNE

    // ====== Flywheel ======
    // Expansion Hub motor ports 2 + 3, 2x goBILDA 5203 bare motor (6000 RPM), same power to both
    public static final String FLYWHEEL_A_MOTOR_NAME = "flywheelA";  // EH M2
    public static final String FLYWHEEL_B_MOTOR_NAME = "flywheelB";  // EH M3
    // Opposite directions assume the motors face each other on one shaft; VERIFY both spin the wheel the same way
    public static final DcMotorSimple.Direction FLYWHEEL_A_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction FLYWHEEL_B_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final int FLYWHEEL_TPR = 28;            // bare motor encoder, ticks per motor rev
    public static double FLYWHEEL_GEAR_RATIO    = 1.0;    // motor revs per wheel rev. PLACEHOLDER
    public static double FLYWHEEL_TARGET_RPM    = 3000.0; // wheel RPM. PLACEHOLDER
    public static double FLYWHEEL_RPM_TOLERANCE = 50.0;
    public static double FLYWHEEL_kP = 0.0005;   // PLACEHOLDER
    public static double FLYWHEEL_kD = 0.0;
    // Feedforward: power = kS + kV * targetRpm
    public static double FLYWHEEL_kS = 0.0;      // power to overcome friction. TUNE: raise until wheel barely starts turning
    public static double FLYWHEEL_kV = 0.00017;  // power per wheel RPM. ~1/6000 at 1:1, ~ratio/6000 once gear ratio known

    // ===== Match Loader =====
    // Control Hub servo port 1, goBILDA positional servo
    public static final String MATCH_LOADER_SERVO_NAME = "matchLoader";
    public static double MATCH_LOADER_DOWN_POS = 0.0;  // TUNE
    public static double MATCH_LOADER_UP_POS   = 0.5;  // TUNE

    // ====== Utilities ======
    public static final double LOOP_HZ = 50.0;
    public static final double dt(ElapsedTime t) { return t.seconds(); }
}
