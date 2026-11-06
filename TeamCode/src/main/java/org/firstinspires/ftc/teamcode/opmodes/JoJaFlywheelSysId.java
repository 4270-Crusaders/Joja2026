package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Flywheel system identification for the Flywheel control loop (rework 2026-11-05), BOTH sides at once.
 * Runs at the same fast loop as JoJa Tele (LOOP_MIN_PERIOD_MS, bulk reads), so what it measures is what the
 * real loop sees.
 *
 * Motor model:  volts = kS + kV * rpm + kA * (rpm/s)
 *
 * Test 1, voltage sweep: holds each voltage in SWEEP_VOLTS until the speed settles, then a straight-line fit of
 *         volts vs rpm gives kS (where the line starts) and kV (its slope).
 * Test 2, step: jumps STEP_FROM_V -> STEP_TO_V and measures
 *         dead time  = how long before the wheel starts reacting (loop + hub + velocity measurement delay)
 *         tau        = how long until it covers 63% of the step
 *         kA         = kV * tau
 * Gains (lambda / IMC tuning of a first-order plant with dead time):
 *         kP = tau * kV / (lambda + deadTime),  kI = kP / tau,  kD = 0 (a flywheel doesn't need it; D adds noise)
 *         lambda = how fast the closed loop should settle (CLOSED_LOOP_TIME_CONSTANT, never under 2 x dead time).
 *
 * Units: the Flywheel loop multiplies its power by (FLYWHEEL_NOMINAL_VOLTAGE / battery), so 1.0 "power" = 12 V.
 * The JoJa numbers are therefore volts / 12, independent of the battery during the test.
 * (2026-11-05 fix: the old SysId divided by the test battery voltage, ~8% too small on a 13 V battery.)
 *
 * Controls: Play, then A = start. B = abort at any time. After the test: Y = apply the results live
 * (until the app restarts), then try them in JoJa Flywheel Tuner and copy them into RoboConstants.
 * Results are also written to the robot log with the tag [SysId].
 *
 * Put the robot somewhere safe: the flywheel spins up to about 10 V. Use a charged battery (12.5 V+).
 */
@TeleOp(name="JoJa Flywheel SysId", group="Test")
public class JoJaFlywheelSysId extends LinearOpMode {
    // ===== Test settings =====
    private static final double[] SWEEP_VOLTS = {2, 3, 4, 5, 6, 7, 8, 9, 10};
    private static final double SETTLE_SECONDS = 2.5;     // per sweep voltage (last AVERAGE_SECONDS are averaged)
    private static final double AVERAGE_SECONDS = 0.5;
    private static final double STEP_FROM_V = 4.0;
    private static final double STEP_TO_V = 9.0;
    private static final double STEP_RECORD_SECONDS = 3.0;
    // How fast the closed loop should settle (seconds). Smaller = more aggressive (more kP / kI), may oscillate.
    private static final double CLOSED_LOOP_TIME_CONSTANT = 0.15;
    private static final double BATTERY_PERIOD_S = 0.05;

    private TelemetryManager panels;
    private Telemetry tele;
    private DcMotorEx left, right;
    private VoltageSensor battery;
    private List<LynxModule> hubs;
    private double batteryVolts = 12.0;
    private final ElapsedTime batteryTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime teleTimer = new ElapsedTime();
    private double loopMsSum = 0.0;
    private int loopCount = 0;

    /** Thrown when B is pressed or the OpMode is stopped. */
    private static class Abort extends RuntimeException { }

    /** Per-side results. */
    private static class Result {
        String name;
        double kS, kV, kA, tau, deadTime, lambda;  // SysId units (volts, RPM, seconds)
        double pS, pV, pA, pP, pI, pD;             // JoJa power units (1.0 = FLYWHEEL_NOMINAL_VOLTAGE)
        boolean ok;
        String error = "";
    }

    @Override
    public void runOpMode() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        left = hardwareMap.get(DcMotorEx.class, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME);
        right = hardwareMap.get(DcMotorEx.class, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME);
        left.setDirection(RoboConstants.FLYWHEEL_LEFT_DIRECTION);
        right.setDirection(RoboConstants.FLYWHEEL_RIGHT_DIRECTION);
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0.0);
        }
        Iterator<VoltageSensor> it = hardwareMap.voltageSensor.iterator();
        battery = it.hasNext() ? it.next() : null;
        readBattery();

        tele.addLine("JoJa Flywheel SysId");
        tele.addLine("Flywheel clear, charged battery. Press Play, then A to start. B aborts.");
        panels.update(telemetry);
        waitForStart();

        while (opModeIsActive() && !gamepad1.a) {
            readBattery();
            tele.addLine("Press A to start the SysId test (about 35 s). B aborts.");
            tele.addData("Battery", "%.2f V", batteryVolts);
            panels.update(telemetry);
            sleep(20);
        }
        if (!opModeIsActive()) return;

        Result l = new Result(), r = new Result();
        l.name = "LEFT";
        r.name = "RIGHT";
        try {
            runTests(l, r);
        } catch (Abort a) {
            stopMotors();
            while (opModeIsActive()) {
                tele.addLine("ABORTED. Nothing was changed. Stop the OpMode.");
                panels.update(telemetry);
                sleep(50);
            }
            return;
        }
        stopMotors();

        boolean applied = false;
        while (opModeIsActive()) {
            if (gamepad1.y && !applied && l.ok && r.ok) {
                apply(l, true);
                apply(r, false);
                applied = true;
                RobotLog.i("[SysId] Applied results to RoboConstants (live until restart).");
            }
            tele.addData("Loop during test", "%.1f ms avg", loopMsSum / Math.max(1, loopCount));
            showResult(l);
            showResult(r);
            tele.addLine(applied ? "APPLIED live. Try them in JoJa Flywheel Tuner, then copy them into RoboConstants."
                    : (l.ok && r.ok) ? "Y = apply these live (until restart). Copy the JoJa numbers into RoboConstants to keep them."
                    : "A side failed: nothing can be applied. Check the motor / encoder and run again.");
            panels.update(telemetry);
            sleep(50);
        }
    }

    // ===================================================================================
    // Tests
    // ===================================================================================

    private void runTests(Result l, Result r) {
        int n = SWEEP_VOLTS.length;
        double[] rpmL = new double[n], rpmR = new double[n];

        // ----- Test 1: steady-state voltage sweep -> kS, kV -----
        for (int i = 0; i < n; i++) {
            double v = SWEEP_VOLTS[i];
            String label = String.format("Sweep %d/%d: %.1f V", i + 1, n, v);
            hold(v, SETTLE_SECONDS - AVERAGE_SECONDS, label, null);
            List<double[]> s = new ArrayList<>();
            hold(v, AVERAGE_SECONDS, label + " (measuring)", s);
            rpmL[i] = average(s, 1);
            rpmR[i] = average(s, 2);
            RobotLog.i(String.format("[SysId] sweep %.1f V -> L %.0f RPM, R %.0f RPM", v, rpmL[i], rpmR[i]));
        }
        fitLine(l, SWEEP_VOLTS, rpmL);
        fitLine(r, SWEEP_VOLTS, rpmR);

        // ----- Test 2: step response -> dead time, tau, kA -----
        String settle = "Step: settling at " + STEP_FROM_V + " V";
        hold(STEP_FROM_V, SETTLE_SECONDS - 0.5, settle, null);
        List<double[]> before = new ArrayList<>();
        hold(STEP_FROM_V, 0.5, settle, before);
        List<double[]> step = new ArrayList<>(); // {t, rpmL, rpmR}
        hold(STEP_TO_V, STEP_RECORD_SECONDS, String.format("Step: %.1f V -> %.1f V", STEP_FROM_V, STEP_TO_V), step);
        stopMotors();

        stepResponse(l, step, 1, average(before, 1));
        stepResponse(r, step, 2, average(before, 2));
        computeGains(l);
        computeGains(r);
        logResult(l);
        logResult(r);
    }

    /** Least-squares fit volts = kS + kV * rpm, using points where the wheel actually turned. */
    private void fitLine(Result res, double[] volts, double[] rpm) {
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        int k = 0;
        for (int i = 0; i < volts.length; i++) {
            if (rpm[i] < 100) continue; // didn't break static friction yet
            sx += rpm[i]; sy += volts[i]; sxx += rpm[i] * rpm[i]; sxy += rpm[i] * volts[i];
            k++;
        }
        double den = k * sxx - sx * sx;
        if (k < 3 || Math.abs(den) < 1e-9) {
            res.ok = false;
            res.error = "not enough moving points (is the motor/encoder plugged in?)";
            return;
        }
        res.kV = (k * sxy - sx * sy) / den;
        res.kS = (sy - res.kV * sx) / k;
        res.ok = res.kV > 0;
        if (!res.ok) res.error = "kV came out <= 0 (check motor direction / encoder)";
        res.kS = Math.max(0.0, res.kS);
    }

    /** Dead time (first 5% of the step) and tau (63.2% point, minus dead time), with interpolation. */
    private void stepResponse(Result res, List<double[]> s, int col, double startRpm) {
        if (!res.ok) return;
        double tEnd = s.get(s.size() - 1)[0];
        double sum = 0;
        int cnt = 0;
        for (double[] p : s) if (p[0] >= tEnd - 0.5) { sum += p[col]; cnt++; }
        double finalRpm = sum / Math.max(1, cnt);
        double delta = finalRpm - startRpm;
        if (delta < 200) {
            res.ok = false;
            res.error = "step barely changed the speed (encoder / motor problem?)";
            return;
        }
        double t5 = crossing(s, col, startRpm + 0.05 * delta);
        double t63 = crossing(s, col, startRpm + 0.632 * delta);
        // 5% point of a pure first-order response is at 0.051 * tau; take that out to get the real dead time
        res.tau = Math.max(0.02, (t63 - t5) / (1.0 - 0.051));
        res.deadTime = Math.max(0.0, t5 - 0.051 * res.tau);
        res.kA = res.kV * res.tau;
    }

    /** First time the speed crosses {@code goal}, interpolated between samples. */
    private double crossing(List<double[]> s, int col, double goal) {
        for (int i = 1; i < s.size(); i++) {
            double[] a = s.get(i - 1), b = s.get(i);
            if (b[col] >= goal) {
                double span = b[col] - a[col];
                double f = (span > 1e-6) ? (goal - a[col]) / span : 1.0;
                return a[0] + Math.max(0.0, Math.min(1.0, f)) * (b[0] - a[0]);
            }
        }
        return s.get(s.size() - 1)[0];
    }

    /** First-order + dead time model -> PI (lambda tuning), then volts -> JoJa power (1.0 = nominal volts). */
    private void computeGains(Result res) {
        if (!res.ok) return;
        res.lambda = Math.max(CLOSED_LOOP_TIME_CONSTANT, 2.0 * res.deadTime);
        double kpVolts = res.tau * res.kV / (res.lambda + res.deadTime); // V per RPM of error
        double kiVolts = kpVolts / res.tau;                               // V per (RPM*s)
        double vNom = RoboConstants.FLYWHEEL_NOMINAL_VOLTAGE;
        res.pS = res.kS / vNom;
        res.pV = res.kV / vNom;
        res.pA = res.kA / vNom;
        res.pP = kpVolts / vNom;
        res.pI = kiVolts / vNom;
        res.pD = 0.0;
    }

    private void apply(Result res, boolean isLeft) {
        if (isLeft) {
            RoboConstants.FLYWHEEL_LEFT_kS = res.pS;
            RoboConstants.FLYWHEEL_LEFT_kV = res.pV;
            RoboConstants.FLYWHEEL_LEFT_kA = res.pA;
            RoboConstants.FLYWHEEL_LEFT_kP = res.pP;
            RoboConstants.FLYWHEEL_LEFT_kI = res.pI;
            RoboConstants.FLYWHEEL_LEFT_kD = res.pD;
        } else {
            RoboConstants.FLYWHEEL_RIGHT_kS = res.pS;
            RoboConstants.FLYWHEEL_RIGHT_kV = res.pV;
            RoboConstants.FLYWHEEL_RIGHT_kA = res.pA;
            RoboConstants.FLYWHEEL_RIGHT_kP = res.pP;
            RoboConstants.FLYWHEEL_RIGHT_kI = res.pI;
            RoboConstants.FLYWHEEL_RIGHT_kD = res.pD;
        }
    }

    // ===================================================================================
    // Output
    // ===================================================================================

    private void showResult(Result res) {
        tele.addLine("====== " + res.name + " ======");
        if (!res.ok) {
            tele.addData("FAILED", res.error);
            return;
        }
        tele.addData("Model kS", "%.3f V", res.kS);
        tele.addData("Model kV", "%.6f V/RPM", res.kV);
        tele.addData("Model kA", "%.6f V/(RPM/s)", res.kA);
        tele.addData("tau / dead time / lambda", "%.3f / %.3f / %.3f s", res.tau, res.deadTime, res.lambda);
        tele.addData("JoJa kS", "%.4f", res.pS);
        tele.addData("JoJa kV", "%.7f", res.pV);
        tele.addData("JoJa kA", "%.7f", res.pA);
        tele.addData("JoJa kP", "%.7f", res.pP);
        tele.addData("JoJa kI", "%.7f", res.pI);
        tele.addData("JoJa kD", "%.1f", res.pD);
    }

    private void logResult(Result res) {
        if (!res.ok) {
            RobotLog.i("[SysId] " + res.name + " FAILED: " + res.error);
            return;
        }
        RobotLog.i(String.format("[SysId] %s model: kS=%.3f V kV=%.6f V/RPM kA=%.6f V/(RPM/s) tau=%.3f s dead=%.3f s lambda=%.3f s loop=%.1f ms",
                res.name, res.kS, res.kV, res.kA, res.tau, res.deadTime, res.lambda, loopMsSum / Math.max(1, loopCount)));
        RobotLog.i(String.format("[SysId] %s JoJa: kS=%.4f kV=%.7f kA=%.7f kP=%.7f kI=%.7f kD=%.1f",
                res.name, res.pS, res.pV, res.pA, res.pP, res.pI, res.pD));
    }

    // ===================================================================================
    // Helpers
    // ===================================================================================

    /**
     * Hold {@code volts} on both motors for {@code seconds} at the fast loop rate.
     * If {@code samples} is not null, records {t, rpmL, rpmR} every loop.
     */
    private void hold(double volts, double seconds, String label, List<double[]> samples) {
        ElapsedTime t = new ElapsedTime();
        while (t.seconds() < seconds) {
            pace();
            checkAbort();
            for (LynxModule hub : hubs) hub.clearBulkCache();
            if (batteryTimer.seconds() >= BATTERY_PERIOD_S) readBattery();
            double p = Math.max(0.0, Math.min(1.0, volts / batteryVolts));
            left.setPower(p);
            right.setPower(p);
            double rl = rpm(left), rr = rpm(right);
            if (samples != null) samples.add(new double[]{t.seconds(), rl, rr});
            if (teleTimer.milliseconds() >= RoboConstants.TELEMETRY_PERIOD_MS) {
                teleTimer.reset();
                tele.addLine(label);
                tele.addData("RPM L", "%.0f", rl);
                tele.addData("RPM R", "%.0f", rr);
                tele.addData("Battery V", "%.2f", batteryVolts);
                tele.addData("Loop ms", "%.1f", loopMsSum / Math.max(1, loopCount));
                tele.addLine("B = abort");
                panels.update(telemetry);
            }
        }
    }

    /** Same steady loop period as LoopTimer (LOOP_MIN_PERIOD_MS). */
    private void pace() {
        double minMs = RoboConstants.LOOP_MIN_PERIOD_MS;
        double waitMs = minMs - loopTimer.milliseconds();
        if (waitMs > 0.5) sleep((long) waitMs);
        while (loopTimer.milliseconds() < minMs) { /* last fraction of a ms */ }
        loopMsSum += loopTimer.milliseconds();
        loopCount++;
        loopTimer.reset();
    }

    private static double average(List<double[]> s, int col) {
        double sum = 0;
        for (double[] p : s) sum += p[col];
        return sum / Math.max(1, s.size());
    }

    private double rpm(DcMotorEx m) {
        return (m.getVelocity() / RoboConstants.FLYWHEEL_TPR) * 60.0 / RoboConstants.FLYWHEEL_GEAR_RATIO;
    }

    private void readBattery() {
        batteryTimer.reset();
        if (battery == null) return;
        double v = battery.getVoltage();
        if (v > 6.0) batteryVolts = v;
    }

    private void stopMotors() {
        left.setPower(0.0);
        right.setPower(0.0);
    }

    private void checkAbort() {
        if (!opModeIsActive() || gamepad1.b) {
            stopMotors();
            throw new Abort();
        }
    }
}
