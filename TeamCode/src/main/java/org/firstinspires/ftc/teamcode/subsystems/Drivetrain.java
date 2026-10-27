package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

import java.util.Iterator;

/**
 * Robot-centric mecanum drivetrain, full speed, no slow mode.
 *
 * States:
 *  IDLE       - all powers 0 (initial state)
 *  MANUAL     - TeleOp stick mix via drive(fwd, strafe, turn)
 *  TIMED_MOVE - Auto: hold (fwd, strafe, turn) for N seconds, voltage-compensated, then IDLE
 */
public class Drivetrain extends SubsystemBase {

    public enum State { IDLE, MANUAL, TIMED_MOVE }

    private final Telemetry telemetry;
    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private final VoltageSensor voltageSensor;

    private State state = State.IDLE;

    // Commanded inputs (fwd +, strafe right +, turn clockwise +)
    private double fwd = 0.0, strafe = 0.0, turn = 0.0;

    // TIMED_MOVE bookkeeping
    private final ElapsedTime moveTimer = new ElapsedTime();
    private double moveSeconds = 0.0;
    private double voltageScale = 1.0;
    private double sampledVoltage = 0.0;

    // Last written wheel powers (for telemetry)
    private double lfPwr = 0.0, lbPwr = 0.0, rfPwr = 0.0, rbPwr = 0.0;

    public Drivetrain(final HardwareMap hMap,
                      final String leftFrontName,
                      final String leftBackName,
                      final String rightFrontName,
                      final String rightBackName,
                      Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = hMap.get(DcMotorEx.class, leftFrontName);
        leftBack = hMap.get(DcMotorEx.class, leftBackName);
        rightFront = hMap.get(DcMotorEx.class, rightFrontName);
        rightBack = hMap.get(DcMotorEx.class, rightBackName);

        leftFront.setDirection(RoboConstants.DRIVE_LEFT_FRONT_DIRECTION);
        leftBack.setDirection(RoboConstants.DRIVE_LEFT_BACK_DIRECTION);
        rightFront.setDirection(RoboConstants.DRIVE_RIGHT_FRONT_DIRECTION);
        rightBack.setDirection(RoboConstants.DRIVE_RIGHT_BACK_DIRECTION);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0.0);
        }

        // Built-in hub voltage sensor (no config name). Null-safe if somehow absent.
        Iterator<VoltageSensor> it = hMap.voltageSensor.iterator();
        voltageSensor = it.hasNext() ? it.next() : null;

        RobotLog.i("[Drivetrain] Initialized.");
    }

    // ---------------- Commands (store targets only) ----------------

    /** TeleOp: robot-centric mecanum drive. Call every loop. IDLE/TIMED_MOVE -> MANUAL. */
    public void drive(double fwd, double strafe, double turn) {
        this.fwd = fwd;
        this.strafe = strafe;
        this.turn = turn;
        state = State.MANUAL;
    }

    /** Auto: hold the commanded powers for {@code seconds}, then IDLE. any -> TIMED_MOVE. */
    public void timedMove(double fwd, double strafe, double turn, double seconds) {
        this.fwd = fwd;
        this.strafe = strafe;
        this.turn = turn;
        this.moveSeconds = seconds;

        // Sample battery voltage once at the start of the move.
        voltageScale = 1.0;
        sampledVoltage = (voltageSensor != null) ? voltageSensor.getVoltage() : 0.0;
        if (RoboConstants.DRIVE_VOLTAGE_COMP_ENABLED && sampledVoltage > 0.0) {
            voltageScale = RoboConstants.DRIVE_NOMINAL_VOLTAGE / sampledVoltage;
        }

        moveTimer.reset();
        state = State.TIMED_MOVE;
    }

    /** any -> IDLE. */
    public void stop() {
        fwd = 0.0;
        strafe = 0.0;
        turn = 0.0;
        state = State.IDLE;
    }

    /** True while a timed move is still running (for the auto sequencer). */
    public boolean isBusy() {
        return state == State.TIMED_MOVE;
    }

    public State getState() {
        return state;
    }

    // ---------------- Hardware writes ----------------

    @Override
    public void periodic() {
        // TIMED_MOVE -> IDLE once the timer has elapsed.
        if (state == State.TIMED_MOVE && moveTimer.seconds() >= moveSeconds) {
            stop();
        }

        double f = 0.0, s = 0.0, t = 0.0;
        switch (state) {
            case MANUAL:
                f = fwd; s = strafe; t = turn;
                break;
            case TIMED_MOVE:
                f = fwd * voltageScale; s = strafe * voltageScale; t = turn * voltageScale;
                break;
            case IDLE:
            default:
                break;
        }

        // Standard mecanum mix
        double lf = f + s + t;
        double lb = f - s + t;
        double rf = f - s - t;
        double rb = f + s - t;

        // Normalize so no wheel exceeds 1.0 (keeps the direction of motion)
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb))));
        lfPwr = lf / max;
        lbPwr = lb / max;
        rfPwr = rf / max;
        rbPwr = rb / max;

        leftFront.setPower(lfPwr);
        leftBack.setPower(lbPwr);
        rightFront.setPower(rfPwr);
        rightBack.setPower(rbPwr);

        telemetry.addLine("====== Drivetrain ======");
        telemetry.addData("Drivetrain/State", state);
        telemetry.addData("Drivetrain/Input", "f=%.2f s=%.2f t=%.2f", fwd, strafe, turn);
        telemetry.addData("Drivetrain/Power", "LF=%.2f LB=%.2f RF=%.2f RB=%.2f", lfPwr, lbPwr, rfPwr, rbPwr);
        if (state == State.TIMED_MOVE) {
            telemetry.addData("Drivetrain/MoveTime", "%.2f / %.2f s", moveTimer.seconds(), moveSeconds);
        }
        telemetry.addData("Drivetrain/VoltScale", "%.3f (%.2f V)", voltageScale, sampledVoltage);
    }
}
