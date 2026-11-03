package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Iterator;

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
        double integral = 0.0, prevErr = 0.0, rpm = 0.0, pwr = 0.0, iPwr = 0.0;
        boolean boosting = false; // full power until close to target

        Side(DcMotorEx motor, boolean isLeft) { this.motor = motor; this.isLeft = isLeft; }

        double kP() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kP : RoboConstants.FLYWHEEL_RIGHT_kP; }
        double kI() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kI : RoboConstants.FLYWHEEL_RIGHT_kI; }
        double kD() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kD : RoboConstants.FLYWHEEL_RIGHT_kD; }
        double kF() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kF : RoboConstants.FLYWHEEL_RIGHT_kF; }
        double kS() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kS : RoboConstants.FLYWHEEL_RIGHT_kS; }

        /** Wheel RPM = motor RPM / gear ratio (gear ratio = motor revs per wheel rev). */
        double readRpm() {
            double motorRpm = (motor.getVelocity() / RoboConstants.FLYWHEEL_TPR) * 60.0;
            return motorRpm / RoboConstants.FLYWHEEL_GEAR_RATIO;
        }

        void reset(double target) {
            // Keep the integral from last time: it's the extra power this side needs (friction),
            // so PIDF starts with it instead of building it up from 0 again after every spin-up.
            boosting = true;
            prevErr = target - readRpm(); // seed so the first derivative term doesn't spike
        }
    }

    Telemetry telemetry;
    private final Side left, right;
    private State state = State.OFF;
    private final VoltageSensor battery;
    private double batteryVolts = 12.0;      // lightly smoothed battery reading
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double dt = 1.0 / RoboConstants.LOOP_HZ; // measured every loop (TeleOp loops slower than the Tuner)

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
        Iterator<VoltageSensor> it = hMap.voltageSensor.iterator();
        battery = it.hasNext() ? it.next() : null;
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

    /** any -> OFF. Power 0. The integral is kept for the next spin-up (see Side.reset). Safe to call every loop. */
    public void stop() {
        state = State.OFF;
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

    private void runSide(Side s, double target) {
        double err = target - s.rpm;

        // ===== Boost: full power from spin-up until within BOOST_BAND of target, once. Then PIDF only. =====
        if (err < RoboConstants.FLYWHEEL_BOOST_BAND_RPM) s.boosting = false;
        if (s.boosting) {
            s.prevErr = err;
            s.pwr = 1.0;
            return;
        }

        // Only build up integral near the target so spin-up from 0 doesn't wind it up and overshoot.
        // This is what lets the higher-friction side close the last gap to the target.
        if (Math.abs(err) <= RoboConstants.FLYWHEEL_I_ZONE_RPM) {
            s.integral += err * dt;
        } else {
            // Hold the integral (don't wipe it): wiping it when a ball hits or the speed swings past the zone
            // dropped the power suddenly and made the +/-300 RPM oscillation worse.
        }
        // Anti-windup: the integral can only add/remove FLYWHEEL_I_MAX_POWER of power
        if (s.kI() > 0) {
            double iMax = RoboConstants.FLYWHEEL_I_MAX_POWER / s.kI();
            s.integral = Math.max(-iMax, Math.min(iMax, s.integral));
        }
        double deriv = (err - s.prevErr) / dt;
        s.prevErr = err;

        s.iPwr = s.kI() * s.integral;
        double ff = (target > 0) ? s.kS() + s.kF() * target : 0.0;
        double pwr = s.kP() * err + s.iPwr + s.kD() * deriv + ff;
        // Voltage compensation: gains are tuned for FLYWHEEL_NOMINAL_VOLTAGE. When the drive/intake sag the battery
        // in TeleOp, scale the power up so the motor still gets the same volts.
        pwr *= RoboConstants.FLYWHEEL_NOMINAL_VOLTAGE / batteryVolts;
        s.pwr = Math.max(0.0, Math.min(1.0, pwr)); // no reverse
    }

    @Override
    public void periodic() {
        double target = RoboConstants.FLYWHEEL_TARGET_RPM;
        left.rpm = left.readRpm();
        right.rpm = right.readRpm();

        // Real loop time, so the integral grows at the same speed in TeleOp (slow loop) as in the Tuner (fast loop)
        dt = Math.max(0.005, Math.min(0.1, loopTimer.seconds()));
        loopTimer.reset();
        if (battery != null) {
            double v = battery.getVoltage();
            if (v > 6.0) batteryVolts = 0.8 * batteryVolts + 0.2 * v; // smooth out spikes
        }

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
        telemetry.addData("Flywheel/Boost L / R", "%s / %s", left.boosting ? "FULL" : "pid", right.boosting ? "FULL" : "pid");
        telemetry.addData("Flywheel/TargetRPM", "%.0f", target);
        // One number per line so Panels can graph each side
        telemetry.addData("Flywheel/RPM L", "%.0f", left.rpm);
        telemetry.addData("Flywheel/RPM R", "%.0f", right.rpm);
        telemetry.addData("Flywheel/Power L", "%.3f", left.pwr);
        telemetry.addData("Flywheel/Power R", "%.3f", right.pwr);
        // How much power kF alone gives, and how much the integral adds (if I power sits at the cap, raise kF)
        telemetry.addData("Flywheel/kF power L", "%.3f", RoboConstants.FLYWHEEL_LEFT_kS + RoboConstants.FLYWHEEL_LEFT_kF * target);
        telemetry.addData("Flywheel/kF power R", "%.3f", RoboConstants.FLYWHEEL_RIGHT_kS + RoboConstants.FLYWHEEL_RIGHT_kF * target);
        telemetry.addData("Flywheel/I power L", "%.3f", left.iPwr);
        telemetry.addData("Flywheel/I power R", "%.3f", right.iPwr);
        telemetry.addData("Flywheel/Loop", "%.0f ms   battery %.2f V", dt * 1000.0, batteryVolts);
    }
}
