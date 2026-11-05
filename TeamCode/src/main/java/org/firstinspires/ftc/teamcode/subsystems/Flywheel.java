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
        double prevRpm = 0.0, accel = 0.0;           // accel = smoothed RPM per second (+ speeding up, - slowing)
        double pPwr = 0.0, vPwr = 0.0, aPwr = 0.0;   // each term's power, for telemetry
        boolean boosting = false;   // full power until close to target
        boolean recovering = false; // full power after a shot slowed it

        Side(DcMotorEx motor, boolean isLeft) { this.motor = motor; this.isLeft = isLeft; }

        double kP() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kP : RoboConstants.FLYWHEEL_RIGHT_kP; }
        double kI() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kI : RoboConstants.FLYWHEEL_RIGHT_kI; }
        double kD() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kD : RoboConstants.FLYWHEEL_RIGHT_kD; }
        double kV() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kV : RoboConstants.FLYWHEEL_RIGHT_kV; }
        double kS() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kS : RoboConstants.FLYWHEEL_RIGHT_kS; }
        double kA() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kA : RoboConstants.FLYWHEEL_RIGHT_kA; }

        /** Smoothed acceleration from the change in RPM since last loop. */
        void updateAccel(double dt) {
            double raw = (rpm - prevRpm) / dt;
            prevRpm = rpm;
            double a = RoboConstants.FLYWHEEL_ACCEL_SMOOTHING;
            accel = a * raw + (1 - a) * accel;
        }

        /** Wheel RPM = motor RPM / gear ratio (gear ratio = motor revs per wheel rev). */
        double readRpm() {
            double motorRpm = (motor.getVelocity() / RoboConstants.FLYWHEEL_TPR) * 60.0;
            return motorRpm / RoboConstants.FLYWHEEL_GEAR_RATIO;
        }

        void reset(double target) {
            // Keep the integral from last time: it's the extra power this side needs (friction),
            // so PIDF starts with it instead of building it up from 0 again after every spin-up.
            // But never carry a NEGATIVE integral into a new spin-up (2026-11-04 stall fix): a negative integral
            // takes power away, so the wheel came out of boost with too little power and sagged / stalled.
            integral = Math.max(0.0, integral);
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

    private boolean shooting = false, wasShooting = false;
    private boolean feedOk = false; // see readyToFeed()
    private boolean feedOverride = false; // paused too long: feed anyway until speed is back
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private double minShotL = 0.0, minShotR = 0.0;

    private static String mode(Side s) {
        return s.boosting ? "FULL (spin-up)" : s.recovering ? "FULL (shot)" : "pid";
    }

    /** true while balls are being fed (stopper open): any speed drop gets full power until it's back. */
    public void setShooting(boolean shooting) { this.shooting = shooting; }

    public double getTargetRpm() { return RoboConstants.FLYWHEEL_TARGET_RPM; }

    /**
     * Feed gate: false while either side is more than FEED_PAUSE below target (a ball just slowed it),
     * true again once both are within FEED_RESUME. Pause the indexer while this is false.
     */
    public boolean readyToFeed() { return feedOk; }

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
        // ===== Shot recovery: while shooting, a POLLEN hitting the wheel slows it. Full power the moment a side
        // is more than RECOVERY_BAND below target, PIDF again once it's back. Keeps both sides from sagging. =====
        if (shooting && err > RoboConstants.FLYWHEEL_RECOVERY_BAND_RPM) {
            s.prevErr = err;
            s.pwr = 1.0;
            s.recovering = true;
            return;
        }
        s.recovering = false;

        // Only build up integral near the target so spin-up from 0 doesn't wind it up and overshoot.
        // This is what lets the higher-friction side close the last gap to the target.
        // 2026-11-04 stall fix: NOT while shooting. Then every dip below RECOVERY_BAND gets full power (above, no
        // integral), so the integral only ever saw the overshoots ABOVE target. It ratcheted down to -I_MAX_POWER,
        // pulled ~0.4 power out, and the wheel stalled / sagged after a volley (and on the next spin-up, since the
        // integral is kept). While shooting, the integral is frozen at its pre-shot value (the friction it learned).
        if (!shooting && Math.abs(err) <= RoboConstants.FLYWHEEL_I_ZONE_RPM) {
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
        s.vPwr = (target > 0) ? s.kS() + s.kV() * target : 0.0;   // velocity feedforward
        s.aPwr = s.kA() * Math.max(0.0, -s.accel);                  // acceleration: push back while it's slowing
        s.pPwr = s.kP() * err;
        double pwr = s.pPwr + s.iPwr + s.kD() * deriv + s.vPwr + s.aPwr;
        // Voltage compensation: gains are tuned for FLYWHEEL_NOMINAL_VOLTAGE. When the drive/intake sag the battery
        // in TeleOp, scale the power up so the motor still gets the same volts.
        pwr *= RoboConstants.FLYWHEEL_NOMINAL_VOLTAGE / batteryVolts;
        s.pwr = Math.max(0.0, Math.min(1.0, pwr)); // no reverse
    }

    @Override
    public void periodic() {
        double target = getTargetRpm();
        left.rpm = left.readRpm();
        right.rpm = right.readRpm();

        // Real loop time, so the integral grows at the same speed in TeleOp (slow loop) as in the Tuner (fast loop)
        dt = Math.max(0.005, Math.min(0.1, loopTimer.seconds()));
        loopTimer.reset();
        left.updateAccel(dt);
        right.updateAccel(dt);
        if (battery != null) {
            double v = battery.getVoltage();
            if (v > 6.0) batteryVolts = 0.8 * batteryVolts + 0.2 * v; // smooth out spikes
        }

        // Feed gate, with a gap between pause and resume so it doesn't flicker on/off
        double worstErr = target - Math.min(left.rpm, right.rpm);
        boolean wasOk = feedOk;
        if (worstErr < RoboConstants.FLYWHEEL_FEED_RESUME_RPM) feedOverride = false; // speed is back: normal again
        if (state == State.OFF) {
            feedOk = false;
            feedOverride = false;
        } else if (worstErr > RoboConstants.FLYWHEEL_FEED_PAUSE_RPM) {
            feedOk = false;
        } else if (worstErr < RoboConstants.FLYWHEEL_FEED_RESUME_RPM) {
            feedOk = true;
        }
        if (wasOk && !feedOk) pauseTimer.reset();
        // Never pause longer than FEED_MAX_PAUSE_S (a ball stuck on the wheel would stall it forever)
        if (!feedOk && state != State.OFF && pauseTimer.seconds() > RoboConstants.FLYWHEEL_FEED_MAX_PAUSE_S) feedOverride = true;
        if (feedOverride) feedOk = true;

        if (state == State.OFF) {
            left.pwr = 0.0;
            right.pwr = 0.0;
        } else {
            runSide(left, target);
            runSide(right, target);
            // Lowest speed during each volley, so you can see if anything still drops toward 3000
            if (shooting && !wasShooting) { minShotL = left.rpm; minShotR = right.rpm; }
            if (shooting) { minShotL = Math.min(minShotL, left.rpm); minShotR = Math.min(minShotR, right.rpm); }
            boolean bothAtSpeed = Math.abs(target - left.rpm) <= RoboConstants.FLYWHEEL_RPM_TOLERANCE
                    && Math.abs(target - right.rpm) <= RoboConstants.FLYWHEEL_RPM_TOLERANCE;
            state = bothAtSpeed ? State.AT_SPEED : State.SPINNING_UP;
        }

        wasShooting = shooting && state != State.OFF;
        left.motor.setPower(left.pwr);
        right.motor.setPower(right.pwr);

        telemetry.addLine("====== Flywheel ======");
        telemetry.addData("Flywheel/State", state);
        telemetry.addData("Flywheel/Boost L / R", "%s / %s", mode(left), mode(right));
        telemetry.addData("Flywheel/Feed", feedOverride ? "GO (pause too long, pushing through)" : feedOk ? "GO" : "WAIT (speed recovering)");
        telemetry.addData("Flywheel/Lowest RPM L (while shooting)", "%.0f", minShotL);
        telemetry.addData("Flywheel/Lowest RPM R (while shooting)", "%.0f", minShotR);
        telemetry.addData("Flywheel/TargetRPM", "%.0f", target);
        // One number per line so Panels can graph each side
        telemetry.addData("Flywheel/RPM L", "%.0f", left.rpm);
        telemetry.addData("Flywheel/RPM R", "%.0f", right.rpm);
        telemetry.addData("Flywheel/Power L", "%.3f", left.pwr);
        telemetry.addData("Flywheel/Power R", "%.3f", right.pwr);
        // How much power kV alone gives, and how much the integral adds (if I power sits at the cap, raise kV)
        // Velocity / acceleration: accel < 0 = slowing down (a POLLEN hit). A big negative spike = how hard the hit was.
        telemetry.addData("Flywheel/Accel L", "%.0f", left.accel);   // RPM per second
        telemetry.addData("Flywheel/Accel R", "%.0f", right.accel);
        // Power from each term (before the battery scaling). Power 1.000 with mode FULL = already maxed out.
        // One number per line, or the Panels graph can't pick it up
        telemetry.addData("Flywheel/kS+kV power L", "%.3f", left.vPwr);
        telemetry.addData("Flywheel/kS+kV power R", "%.3f", right.vPwr);
        telemetry.addData("Flywheel/kA power L", "%.3f", left.aPwr);
        telemetry.addData("Flywheel/kA power R", "%.3f", right.aPwr);
        telemetry.addData("Flywheel/kP power L", "%.3f", left.pPwr);
        telemetry.addData("Flywheel/kP power R", "%.3f", right.pPwr);
        telemetry.addData("Flywheel/I power L", "%.3f", left.iPwr);
        telemetry.addData("Flywheel/I power R", "%.3f", right.iPwr);
        telemetry.addData("Flywheel/Loop ms", "%.0f", dt * 1000.0);
        telemetry.addData("Flywheel/Battery V", "%.2f", batteryVolts);
    }
}
