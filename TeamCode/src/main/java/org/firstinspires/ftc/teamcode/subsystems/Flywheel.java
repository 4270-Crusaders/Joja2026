package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

/** Two flywheel motors, each with its own encoder and its own PDSV velocity loop. */
public class Flywheel extends SubsystemBase {
    public enum State {
        OFF,
        SPINNING_UP,
        AT_SPEED
    }

    /** One motor + encoder + its derivative state. */
    private static class Side {
        final DcMotorEx motor;
        double prevErr = 0.0, rpm = 0.0, err = 0.0, pwr = 0.0;

        Side(HardwareMap hMap, String name, DcMotorSimple.Direction dir) {
            motor = hMap.get(DcMotorEx.class, name);
            motor.setDirection(dir);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // we run our own velocity loop
            motor.setPower(0.0);
        }

        /** Wheel RPM = motor RPM / gear ratio (gear ratio = motor revs per wheel rev). */
        double readRpm() {
            double motorRpm = (motor.getVelocity() / RoboConstants.FLYWHEEL_TPR) * 60.0;
            return motorRpm / RoboConstants.FLYWHEEL_GEAR_RATIO;
        }

        void seed(double target) { prevErr = target - readRpm(); } // first derivative term doesn't spike

        /** One PDSV step. Gains passed in each loop so Panels edits apply live. */
        void control(double target, double kP, double kD, double kS, double kV, double dt) {
            rpm = readRpm();
            err = target - rpm;
            double deriv = (err - prevErr) / dt;
            prevErr = err;
            pwr = kP * err + kD * deriv + kS + kV * target;
            pwr = Math.max(0.0, Math.min(1.0, pwr)); // no reverse
            motor.setPower(pwr);
        }

        void off() {
            rpm = readRpm();
            err = 0.0; prevErr = 0.0; pwr = 0.0;
            motor.setPower(0.0);
        }
    }

    Telemetry telemetry;
    private final Side left, right;
    private State state = State.OFF;

    public Flywheel(final HardwareMap hMap, final String leftName, final String rightName, Telemetry telemetry) {
        this.telemetry = telemetry;
        left = new Side(hMap, leftName, RoboConstants.FLYWHEEL_LEFT_DIRECTION);
        right = new Side(hMap, rightName, RoboConstants.FLYWHEEL_RIGHT_DIRECTION);
        RobotLog.i("[Flywheel] Initialized.");
    }

    /** OFF -> SPINNING_UP. Safe to call every loop; no effect if already spinning. */
    public void spinUp() {
        if (state == State.OFF) {
            left.seed(RoboConstants.FLYWHEEL_TARGET_RPM);
            right.seed(RoboConstants.FLYWHEEL_TARGET_RPM);
            state = State.SPINNING_UP;
        }
    }

    /** any -> OFF. Power 0. Safe to call every loop. */
    public void stop() { state = State.OFF; }

    /** Open-loop power per motor, bypassing the loops. For SysId only; don't call periodic() alongside it. */
    public void setRawPower(double leftPwr, double rightPwr) {
        left.motor.setPower(leftPwr);
        right.motor.setPower(rightPwr);
    }

    public State getState() { return state; }

    public boolean isAtSpeed() { return state == State.AT_SPEED; }

    public double getTargetRpm() { return RoboConstants.FLYWHEEL_TARGET_RPM; }

    public double getLeftRpm() { return left.readRpm(); }

    public double getRightRpm() { return right.readRpm(); }

    double dt = 1.0 / RoboConstants.LOOP_HZ; // FTC OpMode loop ~ 50Hz
    @Override
    public void periodic() {
        double target = RoboConstants.FLYWHEEL_TARGET_RPM;

        if (state == State.OFF) {
            left.off();
            right.off();
        } else {
            left.control(target, RoboConstants.FLYWHEEL_LEFT_kP, RoboConstants.FLYWHEEL_LEFT_kD,
                    RoboConstants.FLYWHEEL_LEFT_kS, RoboConstants.FLYWHEEL_LEFT_kV, dt);
            right.control(target, RoboConstants.FLYWHEEL_RIGHT_kP, RoboConstants.FLYWHEEL_RIGHT_kD,
                    RoboConstants.FLYWHEEL_RIGHT_kS, RoboConstants.FLYWHEEL_RIGHT_kV, dt);

            double tol = RoboConstants.FLYWHEEL_RPM_TOLERANCE;
            state = (Math.abs(left.err) <= tol && Math.abs(right.err) <= tol)
                    ? State.AT_SPEED
                    : State.SPINNING_UP;
        }

        telemetry.addLine("====== Flywheel ======");
        telemetry.addData("Flywheel/State", state);
        telemetry.addData("Flywheel/TargetRPM", "%.0f", target);
        telemetry.addData("Flywheel/Left RPM", "%.0f", left.rpm);
        telemetry.addData("Flywheel/Right RPM", "%.0f", right.rpm);
        telemetry.addData("Flywheel/Left Error", "%.0f", left.err);
        telemetry.addData("Flywheel/Right Error", "%.0f", right.err);
        telemetry.addData("Flywheel/Left Power", "%.2f", left.pwr);
        telemetry.addData("Flywheel/Right Power", "%.2f", right.pwr);
    }
}
