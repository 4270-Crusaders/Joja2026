package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Flywheel SysId. Both motors swept together at the same power (no balls, up to FULL power),
 * each read on its own encoder and fit separately: one kS/kV/kP set per side.
 *  1. Steady-state sweep: hold each power, record settled RPM per side, fit power = kS + kV * rpm.
 *  2. PD estimate from first-order model: kP = kV * (SPEEDUP - 1), kD = 0. No I: kS + kV carry steady state.
 * Results are in power units, valid at the battery voltage shown. Copy them into RoboConstants.
 */
@TeleOp(name = "Flywheel SysId", group = "Tuning")
public class FlywheelSysId extends LinearOpMode {
    private static final double SWEEP_STEP = 0.05;  // power increment, sweeps SWEEP_STEP up to 1.0. 0.05 = 20 points, ~1 min
    private static final double SETTLE_S = 2.5;     // flywheel inertia is high; raise if RPM still climbing
    private static final double SAMPLE_S = 0.5;     // average RPM over this window after settling
    private static final double MIN_RPM = 30.0;     // below this the wheel is stuck on friction; skip point
    private static final double SPEEDUP = 4.0;      // closed loop this many times faster than open loop. Higher = more aggressive kP

    @Override
    public void runOpMode() {
        Flywheel flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME, telemetry);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Flywheel SysId. Remove balls; wheels go to FULL power.");
        telemetry.addLine("Both encoder cables must be plugged in.");
        telemetry.addLine("Press Play to start.");
        telemetry.update();
        waitForStart();
        double startVolts = battery.getVoltage();

        // ===== 1. Steady-state sweep, both sides at once =====
        List<double[]> leftPts = new ArrayList<>(), rightPts = new ArrayList<>(); // {rpm, power}
        int steps = (int) Math.round(1.0 / SWEEP_STEP);
        for (int i = 1; i <= steps; i++) {
            double p = i * SWEEP_STEP; // int counter: no float drift past 1.0
            flywheel.setRawPower(p, p);
            ElapsedTime t = new ElapsedTime();
            while (opModeIsActive() && t.seconds() < SETTLE_S) {
                status(String.format(Locale.US, "Sweep %d/%d: power %.2f  L %d  R %d RPM", i, steps, p,
                        Math.round(flywheel.getLeftRpm()), Math.round(flywheel.getRightRpm())));
            }
            t.reset();
            double sumL = 0, sumR = 0; int n = 0;
            while (opModeIsActive() && t.seconds() < SAMPLE_S) {
                sumL += flywheel.getLeftRpm(); sumR += flywheel.getRightRpm(); n++;
                idle();
            }
            if (!opModeIsActive()) return;
            double rpmL = sumL / Math.max(n, 1), rpmR = sumR / Math.max(n, 1);
            if (rpmL >= MIN_RPM) leftPts.add(new double[]{rpmL, p});
            if (rpmR >= MIN_RPM) rightPts.add(new double[]{rpmR, p});
        }
        flywheel.setRawPower(0.0, 0.0);
        if (leftPts.size() < 2 || rightPts.size() < 2) {
            String bad = leftPts.size() < 2 ? (rightPts.size() < 2 ? "LEFT and RIGHT" : "LEFT") : "RIGHT";
            while (opModeIsActive()) status("SysId FAILED: not enough moving points on " + bad + ". Check encoder cable / direction.");
            return;
        }
        double[] fitL = fitLine(leftPts), fitR = fitLine(rightPts); // power = kS + kV * rpm

        while (opModeIsActive()) {
            telemetry.addLine("===== SysId results =====");
            telemetry.addData("Battery at start", "%.2f V", startVolts);
            show("LEFT", fitL, leftPts);
            show("RIGHT", fitR, rightPts);
            telemetry.addLine("kD = 0 for both (velocity too noisy). kP is a starting point, tune on robot.");
            telemetry.update();
            idle();
        }
    }

    /** Prints one side's constants (named as in RoboConstants) and its sweep table. */
    private void show(String side, double[] fit, List<double[]> pts) {
        double kS = fit[0], kV = fit[1];
        telemetry.addLine("--- " + side + " (" + pts.size() + " points) ---");
        telemetry.addData("FLYWHEEL_" + side + "_kS", "%.5f", kS);
        telemetry.addData("FLYWHEEL_" + side + "_kV", "%.7f", kV);
        telemetry.addData("FLYWHEEL_" + side + "_kP", "%.6f", kV * (SPEEDUP - 1.0)); // PD estimate
        telemetry.addData(side + " max RPM (est)", "%.0f", (1.0 - kS) / kV);
        for (double[] pt : pts) telemetry.addData(String.format(Locale.US, "  %s %.2f", side.charAt(0), pt[1]), "%.0f", pt[0]);
    }

    /** Least squares y = b0 + b1*x over {x, y} points. Returns {b0, b1}. */
    static double[] fitLine(List<double[]> pts) {
        double n = pts.size(), sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double[] p : pts) { sx += p[0]; sy += p[1]; sxx += p[0] * p[0]; sxy += p[0] * p[1]; }
        double b1 = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        return new double[]{(sy - b1 * sx) / n, b1};
    }

    private void status(String msg) {
        telemetry.addLine(msg);
        telemetry.update();
    }
}
