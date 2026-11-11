package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.utils.Battery;
import org.firstinspires.ftc.teamcode.utils.CachedPower;
import org.firstinspires.ftc.teamcode.utils.LoopTimer;

/**
 * Two flywheel motors (one each side of the shooter, not connected). Each side runs its OWN velocity loop with its
 * OWN gains, to the same target RPM. AT_SPEED only when BOTH sides are within tolerance.
 *
 * Control loop (rework 2026-11-05), per side, every loop:
 *
 *   1. Feedforward (rotational motor model, does ~all the work):  ff = kS + kV * target
 *        kS = power to overcome friction, kV = power per RPM. Both from JoJa Flywheel SysId.
 *   2. Feedback (only fixes what the feedforward misses):
 *        P = kP * error
 *        I = kI * integral(error)        only near target, never while shooting, frozen when the output is maxed
 *        D = kD * -(measured accel)      on the measurement, so a target change never kicks it
 *        A = kA * (how fast it's slowing) push back the moment a POLLEN starts slowing the wheel
 *   3. Bang-bang overrides (full power is the fastest way back to speed):
 *        spin-up:  1.0 until within BOOST_BAND of target (once per spin-up)
 *        shooting: 1.0 whenever more than RECOVERY_BAND below target
 *   4. Battery compensation: power * (12 V / battery), clamped 0..1 (never brakes the wheel)
 *
 * Built for a 5-10 ms loop (utils/LoopTimer): speed comes from the bulk cache (no extra hub reads), the battery
 * is read only every 0.1 s (utils/Battery; each read is a separate, slow hub command), motor writes are skipped when
 * the power barely changed, and telemetry is only built on telemetry loops. All filters are in seconds, so they
 * behave the same at any loop speed.
 */
public class Flywheel extends SubsystemBase {
    public enum State {
        OFF,
        SPINNING_UP,
        AT_SPEED
    }

    private enum Mode { OFF, BOOST, RECOVER, PID }


    /** One motor + its own loop state and its own gains (the two sides aren't connected). */
    private static class Side {
        final DcMotorEx motor;
        final CachedPower out;
        final boolean isLeft;
        double rpm = 0.0, prevRpm = 0.0, accel = 0.0; // accel = smoothed RPM per second (+ speeding up, - slowing)
        double integral = 0.0;                        // RPM * s
        double pwr = 0.0;                             // power sent to the motor
        double ffPwr = 0.0, pPwr = 0.0, iPwr = 0.0, dPwr = 0.0, aPwr = 0.0; // each term, for telemetry
        Mode mode = Mode.OFF;
        boolean boosting = false;

        Side(DcMotorEx motor, boolean isLeft) { this.motor = motor; this.out = new CachedPower(motor); this.isLeft = isLeft; }

        double kP() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kP : RoboConstants.FLYWHEEL_RIGHT_kP; }
        double kI() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kI : RoboConstants.FLYWHEEL_RIGHT_kI; }
        double kD() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kD : RoboConstants.FLYWHEEL_RIGHT_kD; }
        double kS() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kS : RoboConstants.FLYWHEEL_RIGHT_kS; }
        double kV() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kV : RoboConstants.FLYWHEEL_RIGHT_kV; }
        double kA() { return isLeft ? RoboConstants.FLYWHEEL_LEFT_kA : RoboConstants.FLYWHEEL_RIGHT_kA; }

        /** Read speed (wheel RPM) from the bulk cache and update the smoothed acceleration. */
        void measure(double dt) {
            double motorRpm = (motor.getVelocity() / RoboConstants.FLYWHEEL_TPR) * 60.0;
            rpm = motorRpm / RoboConstants.FLYWHEEL_GEAR_RATIO;
            double raw = (rpm - prevRpm) / dt;
            prevRpm = rpm;
            // Low-pass filter with a time constant in seconds: same smoothing at any loop speed
            double a = dt / (Math.max(1e-3, RoboConstants.FLYWHEEL_ACCEL_FILTER_S) + dt);
            accel += a * (raw - accel);
        }

        void startSpinUp() {
            // Keep the integral from last time (the extra power this side needs for friction), but never a
            // NEGATIVE one: that takes power away and made the wheel sag / stall after spin-up (2026-11-04).
            integral = Math.max(0.0, integral);
            boosting = true;
        }

        void off() {
            pwr = 0.0;
            ffPwr = pPwr = iPwr = dPwr = aPwr = 0.0;
            mode = Mode.OFF;
        }

        /** One control step. voltScale = 12 V / battery. */
        void run(double target, boolean shooting, double dt, double voltScale) {
            double err = target - rpm;

            // Feedforward + the always-on feedback terms (computed even in bang-bang, so telemetry stays live)
            ffPwr = kS() + kV() * target;
            pPwr = kP() * err;
            dPwr = -kD() * accel;
            aPwr = kA() * Math.max(0.0, -accel);
            iPwr = kI() * integral;

            // 1) Spin-up boost: full power until close to target, once per spin-up
            if (err < RoboConstants.FLYWHEEL_BOOST_BAND_RPM) boosting = false;
            if (boosting) { pwr = 1.0; mode = Mode.BOOST; return; }

            // 2) Shot recovery: a POLLEN slowed it. Full power until it's back within RECOVERY_BAND
            if (shooting && err > RoboConstants.FLYWHEEL_RECOVERY_BAND_RPM) { pwr = 1.0; mode = Mode.RECOVER; return; }

            // 3) Feedforward + PID
            mode = Mode.PID;
            double out = (ffPwr + pPwr + iPwr + dPwr + aPwr) * voltScale;

            // Integral: only near target, never while shooting (it only saw the overshoots and wound negative),
            // and not further into a maxed-out output (anti-windup). Clamped to +/- I_MAX_POWER.
            boolean pushingIntoLimit = (out >= 1.0 && err > 0) || (out <= 0.0 && err < 0);
            if (!shooting && !pushingIntoLimit && Math.abs(err) <= RoboConstants.FLYWHEEL_I_ZONE_RPM && kI() > 0) {
                integral += err * dt;
                double iMax = RoboConstants.FLYWHEEL_I_MAX_POWER / kI();
                integral = Math.max(-iMax, Math.min(iMax, integral));
                double newIPwr = kI() * integral;
                out += (newIPwr - iPwr) * voltScale;
                iPwr = newIPwr;
            }
            pwr = Math.max(0.0, Math.min(1.0, out)); // never reverse / brake the wheel
        }
    }

    Telemetry telemetry;
    private final Side left, right;
    private State state = State.OFF;
    private double batteryVolts = 12.0;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double dt = 1.0 / RoboConstants.LOOP_HZ;

    private boolean shooting = false, wasShooting = false;
    private boolean feedOk = false;       // see readyToFeed()
    private boolean feedOverride = false; // paused too long: feed anyway until speed is back
    private boolean reachedSpeed = false; // both sides got within FEED_RESUME at least once since spin-up
    private double feedScale = 0.0;       // see getFeedScale()
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private double minShotL = 0.0, minShotR = 0.0;
    private boolean leftEnabled = true, rightEnabled = true, telemetryEnabled = true;

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
        Battery.init(hMap); // shared battery reading (utils/Battery)
        batteryVolts = Battery.volts();
        RobotLog.i("[Flywheel] Initialized.");
    }

    /** OFF -> SPINNING_UP. Safe to call every loop; no effect if already spinning. */
    public void spinUp() {
        if (state == State.OFF) {
            left.startSpinUp();
            right.startSpinUp();
            state = State.SPINNING_UP;
        }
    }

    /** any -> OFF. Power 0. The integral is kept for the next spin-up. Safe to call every loop. */
    public void stop() { state = State.OFF; }

    public State getState() { return state; }
    public boolean isAtSpeed() { return state == State.AT_SPEED; }

    /** true while balls are being fed (stopper open): any speed drop gets full power until it's back. */
    public void setShooting(boolean shooting) { this.shooting = shooting; }

    public double getTargetRpm() { return RoboConstants.FLYWHEEL_TARGET_RPM; }

    /**
     * Feed gate: false while either side is more than FEED_PAUSE below target (a ball just slowed it),
     * true again once both are within FEED_RESUME (or after FEED_MAX_PAUSE_S). Slow the indexer while false.
     */
    public boolean readyToFeed() { return feedOk; }

    /** true once both sides have reached speed since the last spin-up (false = still spinning up: don't feed at all). */
    public boolean hasReachedSpeed() { return reachedSpeed; }

    /**
     * Proportional feed (2026-11-05): how hard the feeders should push right now, 0..1.
     *   1 = both wheels within FEED_FULL_RPM of target: feed at full speed
     *   0 = a wheel is FEED_MIN_RPM or more below: feed at the minimum (never a full stop mid-volley)
     *   in between = slowed in proportion to the dip.
     * The indexer and intake use it so they back off exactly as much as the flywheel needs, instead of
     * stop / restart. Less current is taken from the battery while the flywheel recovers, so it recovers faster.
     */
    public double getFeedScale() { return feedScale; }

    public double getRpm() { return 0.5 * (left.rpm + right.rpm); }
    public double getLeftRpm() { return left.rpm; }
    public double getRightRpm() { return right.rpm; }
    public double getLeftPower() { return left.pwr; }
    public double getRightPower() { return right.pwr; }

    // ===== For JoJa Flywheel Graph Test (2026-11-10) =====
    /** Run only some sides (a disabled side gets power 0). Default: both on. */
    public void setSidesEnabled(boolean leftOn, boolean rightOn) { leftEnabled = leftOn; rightEnabled = rightOn; }
    /** false = periodic() adds no telemetry lines (the graph test sends its own). */
    public void setTelemetryEnabled(boolean on) { telemetryEnabled = on; }
    private Side side(boolean isLeft) { return isLeft ? left : right; }
    /** Each control term's power for one side (before battery scaling). */
    public double getFfPower(boolean isLeft) { return side(isLeft).ffPwr; }  // kS + kV * target
    public double getPPower(boolean isLeft)  { return side(isLeft).pPwr; }   // kP * error
    public double getIPower(boolean isLeft)  { return side(isLeft).iPwr; }   // kI * integral
    public double getDPower(boolean isLeft)  { return side(isLeft).dPwr; }   // -kD * accel
    public double getAPower(boolean isLeft)  { return side(isLeft).aPwr; }   // kA * slowing-down accel
    public double getAccel(boolean isLeft)   { return side(isLeft).accel; }  // RPM per second
    /** OFF / BOOST (full power spin-up) / RECOVER (full power after a shot) / PID. */
    public String getMode(boolean isLeft)    { return side(isLeft).mode.name(); }
    public double getBatteryVolts() { return batteryVolts; }

    private void updateFeedGate(double target) {
        double worstErr = target - Math.min(left.rpm, right.rpm);
        boolean wasOk = feedOk;
        if (state == State.OFF) {
            feedOk = false;
            feedOverride = false;
            reachedSpeed = false;
            feedScale = 0.0;
            return;
        }
        if (worstErr < RoboConstants.FLYWHEEL_FEED_RESUME_RPM) { feedOk = true; feedOverride = false; reachedSpeed = true; }
        else if (worstErr > RoboConstants.FLYWHEEL_FEED_PAUSE_RPM) feedOk = false;
        if (wasOk && !feedOk) pauseTimer.reset();
        // Never pause longer than FEED_MAX_PAUSE_S mid-volley (a ball stuck on the wheel would stall it forever).
        // 2026-11-05 fix: only AFTER both sides have reached speed once. Before, during spin-up the pause timer
        // was already old, so this override fired right away and fed balls into a wheel still spinning up.
        if (reachedSpeed && !feedOk && pauseTimer.seconds() > RoboConstants.FLYWHEEL_FEED_MAX_PAUSE_S) feedOverride = true;
        if (feedOverride) feedOk = true;

        // Proportional feed scale
        double full = RoboConstants.FLYWHEEL_FEED_FULL_RPM;
        double span = Math.max(1.0, RoboConstants.FLYWHEEL_FEED_MIN_RPM - full);
        double s = 1.0 - (worstErr - full) / span;
        feedScale = feedOverride ? 1.0 : Math.max(0.0, Math.min(1.0, s));

        // Gate OFF (FLYWHEEL_FEED_GATE_ENABLED = false): feed at full speed the whole time, no waiting
        if (!RoboConstants.FLYWHEEL_FEED_GATE_ENABLED) {
            feedOk = true;
            feedOverride = false;
            reachedSpeed = true;
            feedScale = 1.0;
        }
    }

    @Override
    public void periodic() {
        double target = getTargetRpm();
        // Real loop time, so the integral and accel are right at any loop speed
        dt = Math.max(0.002, Math.min(0.05, loopTimer.seconds()));
        loopTimer.reset();
        left.measure(dt);
        right.measure(dt);
        batteryVolts = Battery.volts(); // shared, re-read every 0.1 s
        double voltScale = RoboConstants.FLYWHEEL_NOMINAL_VOLTAGE / batteryVolts;

        if (state == State.OFF) {
            left.off();
            right.off();
        } else {
            if (leftEnabled) left.run(target, shooting, dt, voltScale); else left.off();
            if (rightEnabled) right.run(target, shooting, dt, voltScale); else right.off();
            // a disabled side (graph test only) doesn't count
            boolean bothAtSpeed = (!leftEnabled || Math.abs(target - left.rpm) <= RoboConstants.FLYWHEEL_RPM_TOLERANCE)
                    && (!rightEnabled || Math.abs(target - right.rpm) <= RoboConstants.FLYWHEEL_RPM_TOLERANCE);
            state = bothAtSpeed ? State.AT_SPEED : State.SPINNING_UP;
            // Lowest speed during each volley
            if (shooting && !wasShooting) { minShotL = left.rpm; minShotR = right.rpm; }
            if (shooting) { minShotL = Math.min(minShotL, left.rpm); minShotR = Math.min(minShotR, right.rpm); }
        }
        updateFeedGate(target);
        wasShooting = shooting && state != State.OFF;

        left.out.set(left.pwr);
        right.out.set(right.pwr);

        if (!telemetryEnabled || !LoopTimer.telemetryNow()) return; // telemetry only every TELEMETRY_PERIOD_MS (it's slow)
        // One number per line, so Panels can graph each one
        telemetry.addLine("====== Flywheel ======");
        telemetry.addData("Flywheel/State", state);
        telemetry.addData("Flywheel/Mode L", left.mode);
        telemetry.addData("Flywheel/Mode R", right.mode);
        telemetry.addData("Flywheel/Feed", feedOverride ? "GO (pause too long, pushing through)" : feedOk ? "GO"
                : reachedSpeed ? "WAIT (speed recovering)" : "WAIT (spinning up)");
        telemetry.addData("Flywheel/Feed scale", "%.2f", feedScale);
        telemetry.addData("Flywheel/TargetRPM", "%.0f", target);
        telemetry.addData("Flywheel/RPM L", "%.0f", left.rpm);
        telemetry.addData("Flywheel/RPM R", "%.0f", right.rpm);
        telemetry.addData("Flywheel/Lowest RPM L (while shooting)", "%.0f", minShotL);
        telemetry.addData("Flywheel/Lowest RPM R (while shooting)", "%.0f", minShotR);
        telemetry.addData("Flywheel/Accel L", "%.0f", left.accel);   // RPM per second, negative = slowing
        telemetry.addData("Flywheel/Accel R", "%.0f", right.accel);
        telemetry.addData("Flywheel/Power L", "%.3f", left.pwr);
        telemetry.addData("Flywheel/Power R", "%.3f", right.pwr);
        // Each term's power (before battery scaling). In PID mode they add up to Power / (12 V / battery).
        telemetry.addData("Flywheel/kS+kV power L", "%.3f", left.ffPwr);
        telemetry.addData("Flywheel/kS+kV power R", "%.3f", right.ffPwr);
        telemetry.addData("Flywheel/kP power L", "%.3f", left.pPwr);
        telemetry.addData("Flywheel/kP power R", "%.3f", right.pPwr);
        telemetry.addData("Flywheel/I power L", "%.3f", left.iPwr);
        telemetry.addData("Flywheel/I power R", "%.3f", right.iPwr);
        telemetry.addData("Flywheel/kD power L", "%.3f", left.dPwr);
        telemetry.addData("Flywheel/kD power R", "%.3f", right.dPwr);
        telemetry.addData("Flywheel/kA power L", "%.3f", left.aPwr);
        telemetry.addData("Flywheel/kA power R", "%.3f", right.aPwr);
        telemetry.addData("Flywheel/Loop ms", "%.0f", dt * 1000.0);
        telemetry.addData("Flywheel/Battery V", "%.2f", batteryVolts);
    }
}
