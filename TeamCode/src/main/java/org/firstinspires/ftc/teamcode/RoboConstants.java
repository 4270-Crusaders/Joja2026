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
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static double INTAKE_POWER         = 1.0;
    public static double INTAKE_REVERSE_POWER = -1.0;

    // ===== Indexer =====
    // Expansion Hub motor port 1, goBILDA 5203 1150 RPM
    public static final String INDEXER_MOTOR_NAME = "indexer";
    public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static double INDEXER_INTAKE_POWER  = 0.6;   // TUNE
    public static double INDEXER_FEED_POWER    = 1.0;
    public static double INDEXER_REVERSE_POWER = -1.0;

    // ===== Stopper =====
    // Control Hub servo port 0, goBILDA positional servo
    public static final String STOPPER_SERVO_NAME = "stopper";
    public static double STOPPER_CLOSED_POS = 0.25;  // TUNE
    public static double STOPPER_OPEN_POS   = 0.0;  // TsUNE

    // ====== Flywheel ======
    // Expansion Hub motor ports 2 + 3, 2x goBILDA 5203 bare motor (6000 RPM).
    // Each motor runs its own PDSV loop on its own encoder: BOTH encoder cables must be plugged in.
    public static final String FLYWHEEL_LEFT_MOTOR_NAME  = "flywheelLeft";   // EH M2
    public static final String FLYWHEEL_RIGHT_MOTOR_NAME = "flywheelRight";  // EH M3
    public static final DcMotorSimple.Direction FLYWHEEL_LEFT_DIRECTION  = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FLYWHEEL_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final int FLYWHEEL_TPR = 28;            // bare motor encoder, ticks per motor rev
    public static double FLYWHEEL_GEAR_RATIO    = 1.0;    // motor revs per wheel rev. PLACEHOLDER
    public static double FLYWHEEL_TARGET_RPM    = 2800; // wheel RPM. PLACEHOLDER
    public static double FLYWHEEL_RPM_TOLERANCE = 100;   // AT_SPEED when BOTH sides are within this
    // Per-side gains. power = kP*err + kD*dErr/dt + kS + kV*targetRpm. Run "Flywheel SysId" to get kS/kV/kP per side.
    public static double FLYWHEEL_LEFT_kP  = 0.000696;
    public static double FLYWHEEL_LEFT_kD  = 0.0;
    public static double FLYWHEEL_LEFT_kS  = 0.33187;
    public static double FLYWHEEL_LEFT_kV  = 0.0002322;
    public static double FLYWHEEL_RIGHT_kP = 0.000565;
    public static double FLYWHEEL_RIGHT_kD = 0.0;
    public static double FLYWHEEL_RIGHT_kS = 0.28076;
    public static double FLYWHEEL_RIGHT_kV = 0.0001883;

    // ===== Match Loader =====
    // Control Hub servo port 1, goBILDA positional servo
    public static final String MATCH_LOADER_SERVO_NAME = "matchLoader";
    public static double MATCH_LOADER_DOWN_POS = 0.5;  // TUNE
    public static double MATCH_LOADER_UP_POS   = 0.0;  // TUNE

    // ====== Utilities ======
    public static final double LOOP_HZ = 50.0;
    public static final double dt(ElapsedTime t) { return t.seconds(); }
}
