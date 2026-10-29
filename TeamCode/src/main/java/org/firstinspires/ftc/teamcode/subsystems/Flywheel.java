package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Two flywheel motors (one each side of the shooter, not connected). Each motor runs its OWN PIDF loop,
 * with its OWN gains, to the same target RPM, so a side with more friction gets more power instead of spinning slower.
 * AT_SPEED only when BOTH sides are within tolerance.
 */
public class Flywheel extends SubsystemBase {
    public enum State {
        OFF,
        SPINNING_UP,
        AT_SPEED
    }

    /** One motor + its own PIDF state and its own gains (the two sides aren't connected). */
    private static class Side {
        final DcMotorEx motor;
        final boolean isLeft;
        double integral = 0.0, prevErr = 0.0, rpm = 0.0, pwr = 0.0;

        Side(DcMotorEx motor, boolean isLeft) { this.motor = motor; this.isLeft = isLeft; }

        double kP() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kP : RoboConstants.FLYWHEEL_RIGHT_kP; }
        double kI() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kI : RoboConstants.FLYWHEEL_RIGHT_kI; }
        double kD() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kD : RoboConstants.FLYWHEEL_RIGHT_kD; }
        double kF() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kF : RoboConstants.FLYWHEEL_RIGHT_kF; }

        /** Wheel RPM = motor RPM / gear ratio (gear ratio = motor revs per wheel rev). */
        double readRpm() {
            double motorRpm = (motor.getVelocity() / RoboConstants.FLYWHEEL_TPR) * 60.0;
            return motorRpm / RoboConstants.FLYWHEEL_GEAR_RATIO;
        }

        void reset(double target) {
            integral = 0.0;
            prevErr = target - readRpm(); // seed so the first derivative term doesn't spike
        }
    }

    Telemetry telemetry;
    private final Side left, right;
    private State state = State.OFF;

    public Flywheel(final HardwareMap hMap, final String flywheelLeftName, final String flywheelRightName, Telemetry telemetry) {
        this.telemetry = telemetry;
        DcMotorEx a = hMap.get(DcMotorEx.class, flywheelLeftName);
        DcMotorEx b = hMap.get(DcMotorEx.class, flywheelRightName);
        a.setDirection(RoboConstants.FLYWHEEL_LEFT_DIRECTION);
        b.setDirection(RoboConstants.FLYWHEEL_RIGHT_DIRECTION);
        for (DcMotorEx m : new DcMotorEx[]{a, b}) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // we run our own velocity loop
            m.setPower(0.0);
        }
        left = new Side(a, true);
        right = new Side(b, false);
        RobotLog.i("[Flywheel] Initialized.");
    }

    /** OFF -> SPINNING_UP. Safe to call every loop; no effect if already spinning. */
    public void spinUp() {
        if (state == State.OFF) {
            left.reset(RoboConstants.FLYWHEEL_TARGET_RPM);
            right.reset(RoboConstants.FLYWHEEL_TARGET_RPM);
            state = State.SPINNING_UP;
        }
    }

    /** any -> OFF. Power 0, integral reset. Safe to call every loop. */
    public void stop() {
        state = State.OFF;
        left.integral = 0.0;
        right.integral = 0.0;
    }

    public State getState() { return state; }

    public boolean isAtSpeed() { return state == State.AT_SPEED; }

    public double getTargetRpm() { return RoboConstants.FLYWHEEL_TARGET_RPM; }

    /** Average wheel RPM of both sides. */
    public double getRpm() { return 0.5 * (left.rpm + right.rpm); }

    public double getLeftRpm() { return left.rpm; }
    public double getRightRpm() { return right.rpm; }
    public double getLeftPower() { return left.pwr; }
    public double getRightPower() { return right.pwr; }

    double dt = 1.0 / RoboConstants.LOOP_HZ; // FTC OpMode loop ~ 50Hz

    private void runSide(Side s, double target) {
        double err = target - s.rpm;
        // Only build up integral near the target so spin-up from 0 doesn't wind it up and overshoot.
        // This is what lets the higher-friction side close the last gap to the target.
        if (Math.abs(err) <= RoboConstants.FLYWHEEL_I_ZONE_RPM) {
            s.integral += err * dt;
        } else {
            s.integral = 0.0;
        }
        double deriv = (err - s.prevErr) / dt;
        s.prevErr = err;

        double pwr = s.kP() * err + s.kI() * s.integral + s.kD() * deriv + s.kF() * target;
        s.pwr = Math.max(0.0, Math.min(1.0, pwr)); // no reverse
    }

    @Override
    public void periodic() {
        double target = RoboConstants.FLYWHEEL_TARGET_RPM;
        left.rpm = left.readRpm();
        right.rpm = right.readRpm();

        if (state == State.OFF) {
            left.pwr = 0.0;
            right.pwr = 0.0;
        } else {
            runSide(left, target);
            runSide(right, target);
            boolean bothAtSpeed = Math.abs(target - left.rpm) <= RoboConstants.FLYWHEEL_RPM_TOLERANCE
                    && Math.abs(target - right.rpm) <= RoboConstants.FLYWHEEL_RPM_TOLERANCE;
            state = bothAtSpeed ? State.AT_SPEED : State.SPINNING_UP;
        }

        left.motor.setPower(left.pwr);
        right.motor.setPower(right.pwr);

        telemetry.addLine("====== Flywheel ======");
        telemetry.addData("Flywheel/State", state);
        telemetry.addData("Flywheel/TargetRPM", "%.0f", target);
        // One number per line so Panels can graph each side
        telemetry.addData("Flywheel/RPM L", "%.0f", left.rpm);
        telemetry.addData("Flywheel/RPM R", "%.0f", right.rpm);
        telemetry.addData("Flywheel/Power L", "%.3f", left.pwr);
        telemetry.addData("Flywheel/Power R", "%.3f", right.pwr);
    }
}
