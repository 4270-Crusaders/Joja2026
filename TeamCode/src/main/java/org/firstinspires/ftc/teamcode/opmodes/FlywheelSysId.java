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
 * Flywheel SysId. Wheel spins freely (no balls) and goes up to full power.
 *  1. Steady-state sweep: hold each power, record settled RPM, fit power = kS + kV * rpm.
 *  2. PD estimate from first-order model: kP = kV * (SPEEDUP - 1), kD = 0. No I: kS + kV carry steady state.
 * Results are in power units, valid at the battery voltage shown. Copy them into RoboConstants.
 */
@TeleOp(name = "Flywheel SysId", group = "Tuning")
public class FlywheelSysId extends LinearOpMode {
    private static final double SWEEP_STEP = 0.05;  // power increment, sweeps SWEEP_STEP..1.0. 0.05 = 20 points, ~1 min
    private static final double SETTLE_S = 2.5;     // flywheel inertia is high; raise if RPM still climbing
    private static final double SAMPLE_S = 0.5;     // average RPM over this window after settling
    private static final double MIN_RPM = 30.0;     // below this the wheel is stuck on friction; skip point
    private static final double SPEEDUP = 4.0;      // closed loop this many times faster than open loop. Higher = more aggressive kP

    @Override
    public void runOpMode() {
        Flywheel flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_A_MOTOR_NAME, RoboConstants.FLYWHEEL_B_MOTOR_NAME, telemetry);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Flywheel SysId. Remove balls; wheel goes to FULL power.");
        telemetry.addLine("Press Play to start.");
        telemetry.update();
        waitForStart();
        double startVolts = battery.getVoltage();

        // ===== 1. Steady-state sweep =====
        List<double[]> pts = new ArrayList<>(); // {rpm, power}
        int steps = (int) Math.round(1.0 / SWEEP_STEP);
        for (int i = 1; i <= steps; i++) {
            double p = i * SWEEP_STEP; // int counter: no float drift past 1.0
            flywheel.setRawPower(p);
            ElapsedTime t = new ElapsedTime();
            while (opModeIsActive() && t.seconds() < SETTLE_S) {
                status(String.format(Locale.US, "Sweep %d/%d: power %.2f  RPM %d", i, steps, p, Math.round(flywheel.getRpm())));
            }
            t.reset();
            double sum = 0; int n = 0;
            while (opModeIsActive() && t.seconds() < SAMPLE_S) {
                sum += flywheel.getRpm(); n++;
                idle();
            }
            if (!opModeIsActive()) return;
            double rpm = sum / Math.max(n, 1);
            if (rpm >= MIN_RPM) pts.add(new double[]{rpm, p});
        }
        if (pts.size() < 2) {
            flywheel.setRawPower(0.0);
            while (opModeIsActive()) status("SysId FAILED: not enough moving points. Check wiring / encoder sign.");
            return;
        }
        double[] fit = fitLine(pts); // power = kS + kV * rpm
        double kS = fit[0], kV = fit[1];

        flywheel.setRawPower(0.0);

        // ===== 2. PD estimate =====
        double kP = kV * (SPEEDUP - 1.0);

        while (opModeIsActive()) {
            telemetry.addLine("===== SysId results =====");
            telemetry.addData("Battery at start", "%.2f V", startVolts);
            telemetry.addData("Points used", pts.size());
            telemetry.addData("FLYWHEEL_kS", "%.5f", kS);
            telemetry.addData("FLYWHEEL_kV", "%.7f", kV);
            telemetry.addData("Max wheel RPM (est)", "%.0f", (1.0 - kS) / kV);
            telemetry.addLine("--- PD estimates (start here, tune on robot) ---");
            telemetry.addData("FLYWHEEL_kP", "%.6f", kP);
            telemetry.addData("FLYWHEEL_kD", "0  (velocity too noisy)");
            telemetry.addLine("--- Sweep (power -> RPM) ---");
            for (double[] pt : pts) telemetry.addData(String.format(Locale.US, "%.2f", pt[1]), "%.0f", pt[0]);
            telemetry.update();
            idle();
        }
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
