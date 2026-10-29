package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import java.util.Iterator;

/**
 * Bench test for JoJa's wiring. Checks every name, port and direction from RoboConstants one device at a time.
 *
 * D-pad up/down   pick a device
 * Motor:  hold A = forward at TEST_POWER, hold B = reverse
 * Servo:  D-pad right/left = +/- 0.05, stick buttons (L/R) = -/+ 0.01, X = go to 0.5
 *         (read the position off telemetry to fill in the *_POS constants)
 * Y               reset all motor encoders
 *
 * Mechanism test (works any time, on top of the above):
 *   Left trigger        intake + indexer pull balls IN (INTAKE_POWER / INDEXER_INTAKE_POWER)
 *   Left stick up       feed: indexer FEED_POWER + intake push balls into the shooter
 *   Left stick down     reverse intake + indexer (unjam)
 *   Right trigger       both flywheel motors, power = how far you press (shows wheel RPM)
 *   R1 (right bumper)   stopper starts CLOSED on init; each press alternates OPEN / CLOSED
 *   L1 (left bumper)    toggle match loader: 1st press DOWN, next UP, ... (MATCH_LOADER_DOWN_POS / MATCH_LOADER_UP_POS)
 *
 * Drive mode (START toggles on/off):
 *   Left stick          drive forward/back + strafe (robot-centric mecanum, like JoJa Tele)
 *   Right stick X       turn
 *   Speed is DRIVE_TEST_SCALE. While drive mode is on, left stick up/down does NOT feed/reverse.
 *
 * Put the robot on a stand first: drive wheels will spin.
 */
@TeleOp(name="JoJa Hardware Test", group="Test")
public class JoJaHardwareTest extends OpMode {
    private TelemetryManager panels;
    private Telemetry tele;
    private static final double TEST_POWER = 0.3;

    // false = servos never move: their signal is turned off so the arms are limp and can be moved by hand.
    // Buttons still change the numbers, which show on telemetry and are written to the robot log ([HWTest]).
    // Set to true to let the servos move again.
    private static final boolean SERVO_OUTPUT_ENABLED = true;

    // true = servo-tuning mode: motors stay OFF and telemetry shows ONLY the servos.
    //   D-pad up/down    pick stopper / match loader
    //   D-pad L/R        -/+ 0.05      stick buttons (L/R)  -/+ 0.01      X  go to 0.5
    //   A                memorize current setpoint as stopper CLOSED / match loader DOWN
    //   B                memorize current setpoint as stopper OPEN   / match loader UP
    //   R1 / L1          toggle stopper / match loader between its two memorized positions
    // Memorized values start from RoboConstants and are logged ([HWTest]); copy them into RoboConstants.
    // Set to false for the full motor + mechanism test.
    private static final boolean SERVO_TUNING_MODE = false;

    // true = flywheel-tuning mode (takes priority over servo-tuning mode). Drive motors stay OFF.
    //   A                PID on/off (holds FLYWHEEL_TARGET_RPM with the real Flywheel code from JoJa Tele)
    //   Right trigger    open-loop raw power to both motors (overrides PID). Press fully = each side's max RPM
    //   D-pad up/down    target +/- 250 RPM      D-pad right/left   target +/- 50 RPM
    //   Y                set target to 85% of the slower side's max seen (do a full-trigger run first)
    //   X                reset the max-seen RPM
    //   R1               stopper OPEN / CLOSED         Left trigger   feed balls (intake + indexer FEED)
    // Gains (kP/kI/kF, tolerance) are live-editable in Panels > Configurables > RoboConstants.
    private static final boolean FLYWHEEL_TUNING_MODE = false; // flywheel tuning now has its own OpMode: JoJa Flywheel Tuner
    private static final double STICK_THRESHOLD = 0.5;
    private static final double DRIVE_TEST_SCALE = 0.6; // drive mode speed (JoJa Tele is full speed)
    // Index into MOTOR_NAMES
    private static final int LEFT_FRONT = 0, LEFT_BACK = 1, RIGHT_FRONT = 2, RIGHT_BACK = 3;

    // Index into MOTOR_NAMES
    private static final int INTAKE = 4, INDEXER = 5, FLY_LEFT = 6, FLY_RIGHT = 7;
    // Index into SERVO_NAMES
    private static final int STOPPER = 0, MATCH_LOADER = 1;

    private static final String[] MOTOR_NAMES = {
            RoboConstants.DRIVE_LEFT_FRONT_NAME,
            RoboConstants.DRIVE_LEFT_BACK_NAME,
            RoboConstants.DRIVE_RIGHT_FRONT_NAME,
            RoboConstants.DRIVE_RIGHT_BACK_NAME,
            RoboConstants.INTAKE_MOTOR_NAME,
            RoboConstants.INDEXER_MOTOR_NAME,
            RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME,
            RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME,
    };
    private static final DcMotorSimple.Direction[] MOTOR_DIRECTIONS = {
            RoboConstants.DRIVE_LEFT_FRONT_DIRECTION,
            RoboConstants.DRIVE_LEFT_BACK_DIRECTION,
            RoboConstants.DRIVE_RIGHT_FRONT_DIRECTION,
            RoboConstants.DRIVE_RIGHT_BACK_DIRECTION,
            RoboConstants.INTAKE_DIRECTION,
            RoboConstants.INDEXER_DIRECTION,
            RoboConstants.FLYWHEEL_LEFT_DIRECTION,
            RoboConstants.FLYWHEEL_RIGHT_DIRECTION,
    };
    // What "forward" (A) should look like on the robot
    private static final String[] MOTOR_EXPECT = {
            "wheel rolls robot FORWARD",
            "wheel rolls robot FORWARD",
            "wheel rolls robot FORWARD",
            "wheel rolls robot FORWARD",
            "pulls balls IN",
            "moves balls toward shooter",
            "shoots balls OUT",
            "shoots balls OUT (same way as A)",
    };
    private static final String[] SERVO_NAMES = {
            RoboConstants.STOPPER_SERVO_NAME,
            RoboConstants.MATCH_LOADER_SERVO_NAME,
    };

    private DcMotorEx[] motors;
    private Servo[] servos;
    private double[] servoPos;
    private double[] lastLogged;
    private IMU imu;
    private VoltageSensor voltageSensor;
    private GamepadEx driverOp;
    private int selected = 0;
    private boolean driveMode = false;
    // Servo-tuning mode: [0] = CLOSED / DOWN, [1] = OPEN / UP, per servo
    private final double[][] memorized = {
            {RoboConstants.STOPPER_CLOSED_POS, RoboConstants.STOPPER_OPEN_POS},
            {RoboConstants.MATCH_LOADER_DOWN_POS, RoboConstants.MATCH_LOADER_UP_POS},
    };
    private final boolean[] atSecond = {false, true}; // false = at CLOSED/DOWN, true = at OPEN/UP. Loader rests UP, so 1st L1 deploys DOWN
    private int tuneSel = 0;
    // Flywheel-tuning mode
    private Flywheel flywheel;
    private boolean flywheelPidOn = false;
    private double maxRpmL = 0.0, maxRpmR = 0.0;
    private boolean stopperOpen = false, matchLoaderUp = true; // first L1 press goes DOWN
    private boolean matchLoaderMoved = false;

    @Override
    public void init() {
        // Panels: subsystems write to `tele`; panels.update(telemetry) sends it to Panels AND the Driver Hub
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        motors = new DcMotorEx[MOTOR_NAMES.length];
        for (int i = 0; i < motors.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setDirection(MOTOR_DIRECTIONS[i]);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servos aren't commanded until you nudge them, so nothing jumps on init
        servos = new Servo[SERVO_NAMES.length];
        servoPos = new double[SERVO_NAMES.length];
        lastLogged = new double[SERVO_NAMES.length];
        java.util.Arrays.fill(lastLogged, Double.NaN);
        for (int i = 0; i < servos.length; i++) {
            servos[i] = hardwareMap.get(Servo.class, SERVO_NAMES[i]);
            servoPos[i] = Double.NaN;
        }
        servos[STOPPER].setDirection(RoboConstants.STOPPER_DIRECTION);
        // Stopper starts CLOSED; R1 then alternates OPEN / CLOSED
        if (!SERVO_OUTPUT_ENABLED) {
            for (Servo sv : servos) {
                if (sv instanceof ServoImplEx) ((ServoImplEx) sv).setPwmDisable(); // limp, free to move by hand
            }
            RobotLog.i("[HWTest] Servo output DISABLED: servos are limp, positions are logged only.");
        }
        writeServo(STOPPER, RoboConstants.STOPPER_CLOSED_POS);
        if (FLYWHEEL_TUNING_MODE) {
            flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME, tele);
        }

        // Same orientation as pedroPathing/Constants: keep the two in sync
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        Iterator<VoltageSensor> it = hardwareMap.voltageSensor.iterator();
        voltageSensor = it.hasNext() ? it.next() : null;

        driverOp = new GamepadEx(gamepad1);

        tele.addData(">", "All devices found. Put the robot on a stand, then press Play.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        driverOp.readButtons();
        if (FLYWHEEL_TUNING_MODE) {
            flywheelTuningLoop();
            return;
        }
        if (SERVO_TUNING_MODE) {
            servoTuningLoop();
            return;
        }
        int total = MOTOR_NAMES.length + SERVO_NAMES.length;

        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) selected = (selected + 1) % total;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP))   selected = (selected + total - 1) % total;
        if (driverOp.wasJustPressed(GamepadKeys.Button.Y)) {
            for (DcMotorEx m : motors) {
                m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        // ===== Mechanism test: NaN = not driven by the mechanism controls this loop =====
        double[] mech = new double[motors.length];
        java.util.Arrays.fill(mech, Double.NaN);
        double leftTrigger = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftStickY = driverOp.getLeftY(); // + = pushed up

        if (driverOp.wasJustPressed(GamepadKeys.Button.START)) driveMode = !driveMode;

        // ===== Drive mode: whole drivetrain, same mecanum mix as Drivetrain.java =====
        if (driveMode) {
            double f = driverOp.getLeftY(), str = driverOp.getLeftX(), t = driverOp.getRightX();
            double lf = f + str + t, lb = f - str + t, rf = f - str - t, rb = f + str - t;
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb))));
            mech[LEFT_FRONT] = lf / max * DRIVE_TEST_SCALE;
            mech[LEFT_BACK] = lb / max * DRIVE_TEST_SCALE;
            mech[RIGHT_FRONT] = rf / max * DRIVE_TEST_SCALE;
            mech[RIGHT_BACK] = rb / max * DRIVE_TEST_SCALE;
        }

        String mechMode = "off";
        if (!driveMode && leftStickY < -STICK_THRESHOLD) {
            mech[INTAKE] = RoboConstants.INTAKE_REVERSE_POWER;
            mech[INDEXER] = RoboConstants.INDEXER_REVERSE_POWER;
            mechMode = "REVERSE (unjam)";
        } else if (!driveMode && leftStickY > STICK_THRESHOLD) {
            mech[INTAKE] = RoboConstants.INTAKE_POWER;
            mech[INDEXER] = indexerFeedPower();
            mechMode = "FEED into shooter";
        } else if (leftTrigger > 0.3) {
            mech[INTAKE] = RoboConstants.INTAKE_POWER;
            mech[INDEXER] = RoboConstants.INDEXER_INTAKE_POWER;
            mechMode = "INTAKE";
        }
        if (rightTrigger > 0.05) {
            mech[FLY_LEFT] = rightTrigger;
            mech[FLY_RIGHT] = rightTrigger;
        }

        // ===== Motors: mechanism controls first, otherwise only the selected one gets power =====
        for (int i = 0; i < motors.length; i++) {
            double pwr = 0.0;
            if (!Double.isNaN(mech[i])) {
                pwr = mech[i];
            } else if (i == selected) {
                if (driverOp.getButton(GamepadKeys.Button.A)) pwr = TEST_POWER;
                else if (driverOp.getButton(GamepadKeys.Button.B)) pwr = -TEST_POWER;
            }
            motors[i].setPower(pwr);
        }

        // ===== Servo toggles: R1 = stopper, L1 = match loader (use the RoboConstants positions) =====
        if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            stopperOpen = !stopperOpen;
            writeServo(STOPPER, stopperOpen ? RoboConstants.STOPPER_OPEN_POS : RoboConstants.STOPPER_CLOSED_POS);
        }
        if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            matchLoaderUp = !matchLoaderUp;
            matchLoaderMoved = true;
            writeServo(MATCH_LOADER, matchLoaderTarget());
        }

        // ===== Servos: fine-tune the selected one =====
        int s = selected - MOTOR_NAMES.length;
        if (s == MATCH_LOADER) {
            // Match loader: nudges change the ZERO OFFSET (shifts UP, DOWN and MIDDLE together), X = offset back to 0.
            // The offset carries over to JoJa Tele until the app restarts; copy it into RoboConstants.MATCH_LOADER_OFFSET.
            double step = 0.0;
            if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))          step = 0.05;
            if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))           step = -0.05;
            if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON))  step = 0.01;
            if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON))   step = -0.01;
            boolean changed = step != 0.0;
            if (driverOp.wasJustPressed(GamepadKeys.Button.X)) { RoboConstants.MATCH_LOADER_OFFSET = 0.0; changed = true; }
            else RoboConstants.MATCH_LOADER_OFFSET += step;
            if (changed) {
                matchLoaderMoved = true;
                writeServo(MATCH_LOADER, matchLoaderTarget());
            }
        } else if (s >= 0) {
            double step = 0.0;
            if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))          step = 0.05;
            if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))           step = -0.05;
            if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON))  step = 0.01;
            if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON))   step = -0.01;
            if (driverOp.wasJustPressed(GamepadKeys.Button.X)) {
                servoPos[s] = 0.5;
            } else if (step != 0.0) {
                double from = Double.isNaN(servoPos[s]) ? 0.5 : servoPos[s];
                servoPos[s] = Math.max(0.0, Math.min(1.0, from + step));
            }
            if (!Double.isNaN(servoPos[s])) writeServo(s, servoPos[s]);
        }

        // ===== Telemetry =====
        if (!SERVO_OUTPUT_ENABLED) tele.addLine("** SERVOS DISABLED (limp): numbers are logged only, nothing moves **");
        tele.addLine("D-pad up/down: pick | A/B: run motor | servo: D-pad L/R, stick buttons, X");
        tele.addData("R1 Stopper setpoint", "%s (%.2f)", stopperOpen ? "OPEN" : "CLOSED",
                stopperOpen ? RoboConstants.STOPPER_OPEN_POS : RoboConstants.STOPPER_CLOSED_POS);
        tele.addData("L1 Match loader setpoint", !matchLoaderMoved ? "not moved yet (press L1)"
                : String.format("%s (%.2f)", matchLoaderUp ? "UP" : "DOWN", matchLoaderTarget()));
        tele.addData("Match loader ZERO OFFSET", "%.2f  (select matchLoader, nudge; X = 0)", RoboConstants.MATCH_LOADER_OFFSET);
        if (s < 0) {
            tele.addData("Selected", "MOTOR %s", MOTOR_NAMES[selected]);
            tele.addData("A should", MOTOR_EXPECT[selected]);
        } else {
            tele.addData("Selected", "SERVO %s", SERVO_NAMES[s]);
            tele.addData("Setpoint", Double.isNaN(servoPos[s]) ? "not moved yet" : String.format("%.2f", servoPos[s]));
        }
        tele.addLine();
        tele.addData("Drive mode (START)", driveMode
                ? String.format("ON  LF %.2f LB %.2f RF %.2f RB %.2f", mech[LEFT_FRONT], mech[LEFT_BACK], mech[RIGHT_FRONT], mech[RIGHT_BACK])
                : "off");
        tele.addLine("====== Mechanism (LT intake | L stick up feed / down reverse | RT shooter) ======");
        tele.addData("Intake + indexer", mechMode);
        double flyTps = 0.5 * (motors[FLY_LEFT].getVelocity() + motors[FLY_RIGHT].getVelocity());
        double flyRpm = flyTps / RoboConstants.FLYWHEEL_TPR * 60.0 / RoboConstants.FLYWHEEL_GEAR_RATIO;
        tele.addData("Shooter power", "%.2f", rightTrigger > 0.05 ? rightTrigger : 0.0);
        tele.addData("Shooter wheel RPM", "%.0f  (L %.0f / R %.0f tps)", flyRpm,
                motors[FLY_LEFT].getVelocity(), motors[FLY_RIGHT].getVelocity());
        tele.addLine();
        tele.addLine("====== Encoders (ticks / ticks per s) ======");
        for (int i = 0; i < motors.length; i++) {
            tele.addData((i == selected ? "> " : "  ") + MOTOR_NAMES[i], "%d  /  %.0f",
                    motors[i].getCurrentPosition(), motors[i].getVelocity());
        }
        tele.addLine("====== Servo setpoints (commanded, NOT measured: goBILDA servos have no position feedback) ======");
        for (int i = 0; i < servos.length; i++) {
            int idx = MOTOR_NAMES.length + i;
            tele.addData((idx == selected ? "> " : "  ") + SERVO_NAMES[i],
                    Double.isNaN(servoPos[i]) ? "-" : String.format("%.2f", servoPos[i]));
        }
        tele.addLine("====== Sensors ======");
        tele.addData("IMU yaw (deg, + = left turn)", "%.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        tele.addData("Battery", "%.2f V", voltageSensor != null ? voltageSensor.getVoltage() : 0.0);
        panels.update(telemetry);
    }

    /** Records the servo number, logs it when it changes, and only moves the servo if output is enabled. */
    /** Same as Indexer FEEDING: about INDEXER_FEED_VOLTS on the motor at the current battery voltage. */
    /** Match loader UP or DOWN position plus the zero offset (same math as MatchLoader.java). */
    private double matchLoaderTarget() {
        double base = matchLoaderUp ? RoboConstants.MATCH_LOADER_UP_POS : RoboConstants.MATCH_LOADER_DOWN_POS;
        return Math.max(0.0, Math.min(1.0, base + RoboConstants.MATCH_LOADER_OFFSET));
    }

    private double indexerFeedPower() {
        double battery = (voltageSensor != null) ? voltageSensor.getVoltage() : 12.0;
        if (battery < 1.0) battery = 12.0;
        return Math.max(0.0, Math.min(1.0, RoboConstants.INDEXER_FEED_VOLTS / battery));
    }

    private void flywheelTuningLoop() {
        // Only the flywheel and the feed path run in this mode
        for (int i = 0; i < motors.length; i++) {
            if (i != FLY_LEFT && i != FLY_RIGHT && i != INTAKE && i != INDEXER) motors[i].setPower(0.0);
        }

        if (driverOp.wasJustPressed(GamepadKeys.Button.A)) flywheelPidOn = !flywheelPidOn;

        double target = RoboConstants.FLYWHEEL_TARGET_RPM;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP))    target += 250;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))  target -= 250;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) target += 50;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))  target -= 50;
        if (driverOp.wasJustPressed(GamepadKeys.Button.X)) { maxRpmL = 0.0; maxRpmR = 0.0; }
        if (driverOp.wasJustPressed(GamepadKeys.Button.Y) && maxRpmL > 0 && maxRpmR > 0) {
            target = Math.round(0.85 * Math.min(maxRpmL, maxRpmR) / 10.0) * 10.0;
        }
        RoboConstants.FLYWHEEL_TARGET_RPM = Math.max(0.0, Math.min(6000.0, target));

        if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            stopperOpen = !stopperOpen;
            writeServo(STOPPER, stopperOpen ? RoboConstants.STOPPER_OPEN_POS : RoboConstants.STOPPER_CLOSED_POS);
        }

        double rt = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        boolean openLoop = rt > 0.05;
        if (flywheelPidOn && !openLoop) flywheel.spinUp(); else flywheel.stop();
        flywheel.periodic(); // reads both encoders, runs the per-side PIDF (or 0), writes power, adds its telemetry
        if (openLoop) {      // raw power wins: written after periodic() in the same loop
            motors[FLY_LEFT].setPower(rt);
            motors[FLY_RIGHT].setPower(rt);
        }

        boolean feeding = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3;
        motors[INTAKE].setPower(feeding ? RoboConstants.INTAKE_POWER : 0.0);
        motors[INDEXER].setPower(feeding ? indexerFeedPower() : 0.0);

        double rpmL = flywheel.getLeftRpm(), rpmR = flywheel.getRightRpm();
        maxRpmL = Math.max(maxRpmL, rpmL);
        maxRpmR = Math.max(maxRpmR, rpmR);

        // ===== Telemetry: flywheel only =====
        tele.addLine("FLYWHEEL TUNING   A: PID on/off | RT: raw power | D-pad: target | Y: 85% of max | X: reset max");
        tele.addLine("R1: stopper | LT: feed balls | gains live in Panels > Configurables");
        tele.addData("Mode", openLoop ? String.format("OPEN LOOP %.2f power", rt) : (flywheelPidOn ? "PID" : "off"));
        tele.addData("Target RPM", "%.0f", RoboConstants.FLYWHEEL_TARGET_RPM);
        tele.addData("At speed (both sides within tol)", flywheel.isAtSpeed());
        tele.addData("Error L", "%.0f", RoboConstants.FLYWHEEL_TARGET_RPM - rpmL);
        tele.addData("Error R", "%.0f", RoboConstants.FLYWHEEL_TARGET_RPM - rpmR);
        tele.addData("Max seen L / R", "%.0f / %.0f", maxRpmL, maxRpmR);
        if (maxRpmL > 0 && maxRpmR > 0 && RoboConstants.FLYWHEEL_TARGET_RPM > 0.95 * Math.min(maxRpmL, maxRpmR)) {
            tele.addLine("!! Target is too close to the slower side's max: it can't hold it. Press Y.");
        }
        if (openLoop && rpmL > 100 && rpmR > 100) {
            // kF here is power per RPM, so a good starting kF = power / RPM it produces
            tele.addData("kF suggest L / R", "%.6f / %.6f", rt / rpmL, rt / rpmR);
        }
        tele.addData("Gains L", "kP %.5f  kI %.5f  kF %.6f", RoboConstants.FLYWHEEL_LEFT_kP,
                RoboConstants.FLYWHEEL_LEFT_kI, RoboConstants.FLYWHEEL_LEFT_kF);
        tele.addData("Gains R", "kP %.5f  kI %.5f  kF %.6f", RoboConstants.FLYWHEEL_RIGHT_kP,
                RoboConstants.FLYWHEEL_RIGHT_kI, RoboConstants.FLYWHEEL_RIGHT_kF);
        tele.addData("Stopper", stopperOpen ? "OPEN" : "CLOSED");
        panels.update(telemetry);
    }

    private void servoTuningLoop() {
        for (DcMotorEx m : motors) m.setPower(0.0); // motors stay off in this mode

        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            tuneSel = 1 - tuneSel;
        }

        // Nudge the selected servo
        double step = 0.0;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))         step = 0.05;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))          step = -0.05;
        if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) step = 0.01;
        if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON))  step = -0.01;
        if (driverOp.wasJustPressed(GamepadKeys.Button.X)) {
            writeServo(tuneSel, 0.5);
        } else if (step != 0.0) {
            double from = Double.isNaN(servoPos[tuneSel]) ? 0.5 : servoPos[tuneSel];
            writeServo(tuneSel, Math.max(0.0, Math.min(1.0, from + step)));
        }

        // Memorize the current setpoint
        String[][] names = {{"CLOSED", "OPEN"}, {"DOWN", "UP"}};
        if (!Double.isNaN(servoPos[tuneSel])) {
            if (driverOp.wasJustPressed(GamepadKeys.Button.A)) {
                memorized[tuneSel][0] = servoPos[tuneSel];
                atSecond[tuneSel] = false;
                RobotLog.i("[HWTest] MEMORIZED %s %s = %.3f", SERVO_NAMES[tuneSel], names[tuneSel][0], servoPos[tuneSel]);
            }
            if (driverOp.wasJustPressed(GamepadKeys.Button.B)) {
                memorized[tuneSel][1] = servoPos[tuneSel];
                atSecond[tuneSel] = true;
                RobotLog.i("[HWTest] MEMORIZED %s %s = %.3f", SERVO_NAMES[tuneSel], names[tuneSel][1], servoPos[tuneSel]);
            }
        }

        // Toggle between the two memorized positions
        if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            atSecond[STOPPER] = !atSecond[STOPPER];
            writeServo(STOPPER, memorized[STOPPER][atSecond[STOPPER] ? 1 : 0]);
        }
        if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            atSecond[MATCH_LOADER] = !atSecond[MATCH_LOADER];
            writeServo(MATCH_LOADER, memorized[MATCH_LOADER][atSecond[MATCH_LOADER] ? 1 : 0]);
        }

        // ===== Telemetry: servos only =====
        tele.addLine("SERVO TUNING  (setpoints: goBILDA servos have no position feedback)");
        tele.addLine("D-pad U/D pick | D-pad L/R +-0.05 | stick btns +-0.01 | X 0.5");
        tele.addLine("A = save 1st pos | B = save 2nd pos | R1 stopper / L1 loader toggle");
        tele.addLine();
        for (int i = 0; i < SERVO_NAMES.length; i++) {
            tele.addLine((i == tuneSel ? "> " : "  ") + SERVO_NAMES[i]);
            tele.addData("    setpoint", Double.isNaN(servoPos[i]) ? "not moved yet" : String.format("%.3f", servoPos[i]));
            tele.addData("    " + names[i][0] + " (A)", "%.3f%s", memorized[i][0], !atSecond[i] ? "  <- toggle here" : "");
            tele.addData("    " + names[i][1] + " (B)", "%.3f%s", memorized[i][1], atSecond[i] ? "  <- toggle here" : "");
        }
        panels.update(telemetry);
    }

    private void writeServo(int i, double pos) {
        servoPos[i] = pos;
        if (Double.isNaN(lastLogged[i]) || Math.abs(lastLogged[i] - pos) > 1e-9) {
            RobotLog.i("[HWTest] %s = %.3f%s", SERVO_NAMES[i], pos, SERVO_OUTPUT_ENABLED ? "" : " (not sent: output disabled)");
            lastLogged[i] = pos;
        }
        if (SERVO_OUTPUT_ENABLED) servos[i].setPosition(pos);
    }

    @Override
    public void stop() {
        for (DcMotorEx m : motors) m.setPower(0.0);
    }
}
