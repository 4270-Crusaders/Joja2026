package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
 * Flywheel system identification (SysId) for BOTH flywheel motors at once (each side gets its own numbers).
 *
 * Test 1, voltage sweep: holds each voltage in SWEEP_VOLTS until the speed settles, then fits
 *         volts = kS + kV * rpm   (straight line through the steady-state points)
 * Test 2, step: jumps from STEP_FROM_V to STEP_TO_V and times how long the wheel takes to get 63% of the way
 *         (time constant tau). kA = kV * tau.
 * Gains: PI from the first-order model (lambda / IMC tuning). kD = 0: a flywheel doesn't need it and D only adds noise.
 *
 * Results are shown in two forms:
 *   - SysId units (volts, RPM): kS [V], kV [V/RPM], kA [V/(RPM/s)]
 *   - JoJa Flywheel units (motor power 0..1, at the battery voltage measured during the test):
 *     kS, kF, kP, kI, kD -> what goes into RoboConstants.FLYWHEEL_LEFT_* / FLYWHEEL_RIGHT_*
 *
 * Controls: Play, then A = start. B = abort at any time. After the test: Y = apply the results live
 * (until the app restarts) so you can try them right away in JoJa Flywheel Tuner.
 * Results are also written to the robot log with the tag [SysId].
 *
 * Put the robot somewhere safe: the flywheel spins up to about 10 V. Use a charged battery (12.5 V+).
 */
@TeleOp(name="JoJa Flywheel SysId", group="Test")
public class JoJaFlywheelSysId extends LinearOpMode {
    // ===== Test settings =====
    private static final double[] SWEEP_VOLTS = {3, 4, 5, 6, 7, 8, 9, 10};
    private static final double SETTLE_SECONDS = 2.5;     // per sweep voltage (last AVERAGE_SECONDS are averaged)
    private static final double AVERAGE_SECONDS = 0.5;
    private static final double STEP_FROM_V = 4.0;
    private static final double STEP_TO_V = 8.0;
    private static final double STEP_RECORD_SECONDS = 3.0;
    // PI tuning: how fast the closed loop should respond (seconds). Smaller = more aggressive, may oscillate.
    private static final double CLOSED_LOOP_TIME_CONSTANT = 0.25;
    private static final double DEAD_TIME = 0.03;          // loop + hub delay, seconds

    private TelemetryManager panels;
    private Telemetry tele;
    private DcMotorEx left, right;
    private VoltageSensor battery;
    private double batterySum = 0.0;
    private int batteryCount = 0;

    /** Thrown when B is pressed or the OpMode is stopped. */
    private static class Abort extends RuntimeException { }

    /** Per-side results. */
    private static class Result {
        String name;
        double kS, kV, kA, tau;          // SysId units
        double pS, pF, pP, pI, pD;       // JoJa power units
        boolean ok;
        String error = "";
    }

    @Override
    public void runOpMode() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();

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

        tele.addLine("JoJa Flywheel SysId");
        tele.addLine("Flywheel clear, charged battery. Press Play, then A to start. B aborts.");
        panels.update(telemetry);
        waitForStart();

        while (opModeIsActive() && !gamepad1.a) {
            tele.addLine("Press A to start the SysId test (about 30 s). B aborts.");
            tele.addData("Battery", "%.2f V", batteryVolts());
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
            showResult(l);
            showResult(r);
            tele.addLine(applied ? "APPLIED live. Try them in JoJa Flywheel Tuner, then copy them into RoboConstants."
                    : "Y = apply these live (until restart). Copy the JoJa numbers into RoboConstants to keep them.");
            panels.update(telemetry);
            sleep(50);
        }
    }

    // ===================================================================================
    // Tests
    // ===================================================================================

    private void runTests(Result l, Result r) {
        int n = SWEEP_VOLTS.length;
        double[] vL = new double[n], vR = new double[n], rpmL = new double[n], rpmR = new double[n];

        // ----- Test 1: steady-state voltage sweep -----
        for (int i = 0; i < n; i++) {
            double v = SWEEP_VOLTS[i];
            String label = String.format("Sweep %d/%d: %.1f V", i + 1, n, v);
            holdVolts(v, SETTLE_SECONDS - AVERAGE_SECONDS, label);
            double[] avg = averageRpm(v, AVERAGE_SECONDS, label);
            vL[i] = v; vR[i] = v;
            rpmL[i] = avg[0]; rpmR[i] = avg[1];
            RobotLog.i(String.format("[SysId] sweep %.1f V -> L %.0f RPM, R %.0f RPM", v, avg[0], avg[1]));
        }
        fitLine(l, vL, rpmL);
        fitLine(r, vR, rpmR);

        // ----- Test 2: step response -----
        holdVolts(STEP_FROM_V, SETTLE_SECONDS - 0.3, "Step: settling at " + STEP_FROM_V + " V");
        double[] start = averageRpm(STEP_FROM_V, 0.3, "Step: settling at " + STEP_FROM_V + " V");

        List<double[]> samples = new ArrayList<>(); // {t, rpmL, rpmR}
        ElapsedTime t = new ElapsedTime();
        while (t.seconds() < STEP_RECORD_SECONDS) {
            checkAbort();
            setVolts(STEP_TO_V);
            samples.add(new double[]{t.seconds(), rpm(left), rpm(right)});
            tele.addLine(String.format("Step: %.1f V -> %.1f V  (%.1f s)", STEP_FROM_V, STEP_TO_V, t.seconds()));
            panels.update(telemetry);
            sleep(5);
        }
        stopMotors();

        l.tau = timeConstant(samples, 1, start[0]);
        r.tau = timeConstant(samples, 2, start[1]);
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
            if (rpm[i] < 50) continue; // didn't break static friction yet
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
    }

    /** Time to cover 63.2% of the step, final speed = average of the last 0.5 s. */
    private double timeConstant(List<double[]> s, int col, double startRpm) {
        double tEnd = s.get(s.size() - 1)[0];
        double sum = 0;
        int cnt = 0;
        for (double[] p : s) if (p[0] >= tEnd - 0.5) { sum += p[col]; cnt++; }
        double finalRpm = sum / Math.max(1, cnt);
        double goal = startRpm + 0.632 * (finalRpm - startRpm);
        for (double[] p : s) if (p[col] >= goal) return Math.max(p[0], 0.02);
        return tEnd; // never got there; shouldn't happen
    }

    /** First-order model PI (lambda tuning), then convert volts -> motor power at the test battery voltage. */
    private void computeGains(Result res) {
        if (!res.ok) return;
        res.kA = res.kV * res.tau;
        double vBatt = averageBattery();
        // plant: rpm/volt = 1/kV with time constant tau
        double kpVolts = res.tau * res.kV / (CLOSED_LOOP_TIME_CONSTANT + DEAD_TIME); // V per RPM of error
        double kiVolts = kpVolts / res.tau;                                           // V per (RPM*s)
        res.pS = res.kS / vBatt;
        res.pF = res.kV / vBatt;
        res.pP = kpVolts / vBatt;
        res.pI = kiVolts / vBatt;
        res.pD = 0.0;
    }

    private void apply(Result res, boolean isLeft) {
        if (isLeft) {
            RoboConstants.FLYWHEEL_LEFT_kS = res.pS;
            RoboConstants.FLYWHEEL_LEFT_kF = res.pF;
            RoboConstants.FLYWHEEL_LEFT_kP = res.pP;
            RoboConstants.FLYWHEEL_LEFT_kI = res.pI;
            RoboConstants.FLYWHEEL_LEFT_kD = res.pD;
        } else {
            RoboConstants.FLYWHEEL_RIGHT_kS = res.pS;
            RoboConstants.FLYWHEEL_RIGHT_kF = res.pF;
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
        tele.addData("SysId kS", "%.3f V", res.kS);
        tele.addData("SysId kV", "%.6f V/RPM", res.kV);
        tele.addData("SysId kA", "%.6f V/(RPM/s)  (tau %.2f s)", res.kA, res.tau);
        tele.addData("JoJa kS", "%.4f", res.pS);
        tele.addData("JoJa kF", "%.7f", res.pF);
        tele.addData("JoJa kP", "%.7f", res.pP);
        tele.addData("JoJa kI", "%.7f", res.pI);
        tele.addData("JoJa kD", "%.1f", res.pD);
    }

    private void logResult(Result res) {
        if (!res.ok) {
            RobotLog.i("[SysId] " + res.name + " FAILED: " + res.error);
            return;
        }
        RobotLog.i(String.format("[SysId] %s SysId: kS=%.3f V kV=%.6f V/RPM kA=%.6f V/(RPM/s) tau=%.3f s (battery %.2f V)",
                res.name, res.kS, res.kV, res.kA, res.tau, averageBattery()));
        RobotLog.i(String.format("[SysId] %s JoJa: kS=%.4f kF=%.7f kP=%.7f kI=%.7f kD=%.1f",
                res.name, res.pS, res.pF, res.pP, res.pI, res.pD));
    }

    // ===================================================================================
    // Helpers
    // ===================================================================================

    private void holdVolts(double volts, double seconds, String label) {
        ElapsedTime t = new ElapsedTime();
        while (t.seconds() < seconds) {
            checkAbort();
            setVolts(volts);
            tele.addLine(label);
            tele.addData("RPM L / R", "%.0f / %.0f", rpm(left), rpm(right));
            tele.addLine("B = abort");
            panels.update(telemetry);
            sleep(10);
        }
    }

    /** Hold the voltage and average both RPMs over {@code seconds}. */
    private double[] averageRpm(double volts, double seconds, String label) {
        double sL = 0, sR = 0;
        int cnt = 0;
        ElapsedTime t = new ElapsedTime();
        while (t.seconds() < seconds) {
            checkAbort();
            setVolts(volts);
            sL += rpm(left);
            sR += rpm(right);
            cnt++;
            tele.addLine(label + " (measuring)");
            panels.update(telemetry);
            sleep(10);
        }
        return new double[]{sL / Math.max(1, cnt), sR / Math.max(1, cnt)};
    }

    /** Same voltage on both motors, compensated for the current battery voltage. */
    private void setVolts(double volts) {
        double vb = batteryVolts();
        batterySum += vb;
        batteryCount++;
        double p = Math.max(0.0, Math.min(1.0, volts / vb));
        left.setPower(p);
        right.setPower(p);
    }

    private double rpm(DcMotorEx m) {
        return (m.getVelocity() / RoboConstants.FLYWHEEL_TPR) * 60.0 / RoboConstants.FLYWHEEL_GEAR_RATIO;
    }

    private double batteryVolts() {
        double v = (battery != null) ? battery.getVoltage() : 12.0;
        return (v > 1.0) ? v : 12.0;
    }

    private double averageBattery() {
        return batteryCount > 0 ? batterySum / batteryCount : batteryVolts();
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
