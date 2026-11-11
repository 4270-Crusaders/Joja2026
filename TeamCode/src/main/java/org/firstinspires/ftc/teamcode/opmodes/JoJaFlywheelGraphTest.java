package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.utils.LoopTimer;

import java.util.List;

/**
 * Flywheel graph test (2026-11-10): graphs each flywheel's RPM, target RPM and every PIDF term in Panels.
 * Same Flywheel class + gains as JoJa Tele. Edit FLYWHEEL_LEFT_* / FLYWHEEL_RIGHT_* live in
 * Panels > Configurables > RoboConstants and watch the graph change.
 *
 * Panels > Graph: pick the lines to show. RPM lines are in the thousands and power lines are 0..1,
 * so graph them separately (e.g. "L RPM" + "L Target RPM", then "L Power" + "L kS+kV" + "L kP" + "L kI").
 *
 *   A                 flywheel ON / OFF
 *   X                 which side runs: BOTH -> LEFT only -> RIGHT only
 *   D-pad up/down     target +/- 100 RPM      D-pad right/left   target +/- 25 RPM
 *   Y                 step test ON / OFF: target jumps between target and (target - STEP_RPM) every STEP_PERIOD_S
 *                     (shows rise time, overshoot and how it settles: like a ball hitting the wheel)
 */
@Configurable
@TeleOp(name="JoJa Flywheel Graph Test", group="Test")
public class JoJaFlywheelGraphTest extends OpMode {
    public static double STEP_RPM = 500;
    public static double STEP_PERIOD_S = 2.0;

    private enum Sides { BOTH, LEFT, RIGHT }

    private TelemetryManager panels;
    private Telemetry tele;
    private Flywheel flywheel;
    private GamepadEx driverOp;
    private List<LynxModule> hubs;
    private LoopTimer loopTimer;
    private final ElapsedTime stepTimer = new ElapsedTime();

    private boolean on = false, stepTest = false, stepLow = false;
    private Sides sides = Sides.BOTH;
    private double baseTarget;     // the target you set (D-pad / Panels)
    private double lastWritten;    // what this test last wrote into FLYWHEEL_TARGET_RPM

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME, tele);
        flywheel.setTelemetryEnabled(false); // this test sends its own (graph-friendly) lines
        driverOp = new GamepadEx(gamepad1);
        baseTarget = lastWritten = RoboConstants.FLYWHEEL_TARGET_RPM;

        tele.addLine("JoJa Flywheel Graph Test. A = on/off, X = side, D-pad = target, Y = step test. Press Play.");
        panels.update(telemetry);
    }

    @Override
    public void start() { loopTimer = new LoopTimer(); }

    @Override
    public void loop() {
        loopTimer.tick();
        for (LynxModule hub : hubs) hub.clearBulkCache();
        driverOp.readButtons();

        // Changed in Panels? Use that as the new target.
        if (RoboConstants.FLYWHEEL_TARGET_RPM != lastWritten) baseTarget = RoboConstants.FLYWHEEL_TARGET_RPM;

        if (driverOp.wasJustPressed(GamepadKeys.Button.A)) on = !on;
        if (driverOp.wasJustPressed(GamepadKeys.Button.X)) {
            sides = Sides.values()[(sides.ordinal() + 1) % Sides.values().length];
            flywheel.stop(); // restart the spin-up for the new side(s)
        }
        if (driverOp.wasJustPressed(GamepadKeys.Button.Y)) { stepTest = !stepTest; stepLow = false; stepTimer.reset(); }
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP))    baseTarget += 100;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))  baseTarget -= 100;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) baseTarget += 25;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))  baseTarget -= 25;
        baseTarget = Math.max(0.0, Math.min(6000.0, baseTarget));

        if (stepTest && stepTimer.seconds() >= STEP_PERIOD_S) { stepLow = !stepLow; stepTimer.reset(); }
        double target = (stepTest && stepLow) ? Math.max(0.0, baseTarget - STEP_RPM) : baseTarget;
        RoboConstants.FLYWHEEL_TARGET_RPM = target; // the Flywheel reads its target from here
        lastWritten = target;

        flywheel.setSidesEnabled(sides != Sides.RIGHT, sides != Sides.LEFT);
        if (on) flywheel.spinUp(); else flywheel.stop();
        flywheel.periodic();

        if (!LoopTimer.telemetryNow()) return; // ~20 graph points per second

        // ===== Panels graph lines: one number each =====
        graphSide("L", true, target);
        graphSide("R", false, target);
        panels.addData("Battery V", round(flywheel.getBatteryVolts(), 100));

        // ===== Driver Hub text =====
        tele.addData("Flywheel", on ? "ON" : "OFF (press A)");
        tele.addData("Running", sides);
        tele.addData("Step test", stepTest ? String.format("ON: %.0f <-> %.0f every %.1f s", baseTarget,
                Math.max(0.0, baseTarget - STEP_RPM), STEP_PERIOD_S) : "OFF (press Y)");
        tele.addData("Mode L / R", "%s / %s", flywheel.getMode(true), flywheel.getMode(false));
        tele.addData("Left  kS/kV/kA", "%.4f  %.7f  %.7f", RoboConstants.FLYWHEEL_LEFT_kS,
                RoboConstants.FLYWHEEL_LEFT_kV, RoboConstants.FLYWHEEL_LEFT_kA);
        tele.addData("Left  kP/kI/kD", "%.7f  %.7f  %.7f", RoboConstants.FLYWHEEL_LEFT_kP,
                RoboConstants.FLYWHEEL_LEFT_kI, RoboConstants.FLYWHEEL_LEFT_kD);
        tele.addData("Right kS/kV/kA", "%.4f  %.7f  %.7f", RoboConstants.FLYWHEEL_RIGHT_kS,
                RoboConstants.FLYWHEEL_RIGHT_kV, RoboConstants.FLYWHEEL_RIGHT_kA);
        tele.addData("Right kP/kI/kD", "%.7f  %.7f  %.7f", RoboConstants.FLYWHEEL_RIGHT_kP,
                RoboConstants.FLYWHEEL_RIGHT_kI, RoboConstants.FLYWHEEL_RIGHT_kD);
        tele.addData("Loop avg / max ms", "%.1f / %.1f", loopTimer.getAverageMs(), loopTimer.getMaxMs());
        panels.update(telemetry);
    }

    /** All graph lines for one side. Power terms are before battery scaling (they add up in PID mode). */
    private void graphSide(String s, boolean isLeft, double target) {
        double rpm = isLeft ? flywheel.getLeftRpm() : flywheel.getRightRpm();
        boolean running = isLeft ? sides != Sides.RIGHT : sides != Sides.LEFT;
        panels.addData(s + " Target RPM", running && on ? target : 0.0);
        panels.addData(s + " RPM", round(rpm, 1));
        panels.addData(s + " Error RPM", round(running && on ? target - rpm : 0.0, 1));
        panels.addData(s + " Power", round(isLeft ? flywheel.getLeftPower() : flywheel.getRightPower(), 1000));
        panels.addData(s + " kS+kV", round(flywheel.getFfPower(isLeft), 1000));
        panels.addData(s + " kP", round(flywheel.getPPower(isLeft), 1000));
        panels.addData(s + " kI", round(flywheel.getIPower(isLeft), 1000));
        panels.addData(s + " kD", round(flywheel.getDPower(isLeft), 1000));
        panels.addData(s + " kA", round(flywheel.getAPower(isLeft), 1000));
        panels.addData(s + " Accel RPM/s", round(flywheel.getAccel(isLeft), 1));
    }

    private static double round(double v, double per) { return Math.round(v * per) / per; }

    @Override
    public void stop() {
        RoboConstants.FLYWHEEL_TARGET_RPM = baseTarget; // don't leave the step-test low target behind
        flywheel.stop();
        flywheel.periodic();
        LoopTimer.release();
    }
}
