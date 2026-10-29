package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

/**
 * Flywheel tuning only. Uses the same Flywheel class as JoJa Tele, so what you tune here is what you get.
 * Each side has its own PIDF (the motors aren't connected): edit FLYWHEEL_LEFT_* / FLYWHEEL_RIGHT_* and
 * FLYWHEEL_TARGET_RPM live in Panels > Configurables > RoboConstants.
 *
 * A                  flywheel ON / OFF
 * D-pad up/down      target velocity +/- 100 RPM
 * D-pad right/left   target velocity +/- 25 RPM
 */
@TeleOp(name="JoJa Flywheel Tuner", group="Test")
public class JoJaFlywheelTuner extends OpMode {
    private TelemetryManager panels;
    private Telemetry tele;

    private Flywheel flywheel;
    private GamepadEx driverOp;
    private boolean on = false;

    @Override
    public void init() {
        // Panels: tele goes to Panels AND the Driver Hub on panels.update(telemetry)
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME, tele);
        driverOp = new GamepadEx(gamepad1);

        tele.addData(">", "Flywheel Tuner ready. A = on/off, D-pad = target RPM. Press Play.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        driverOp.readButtons();

        if (driverOp.wasJustPressed(GamepadKeys.Button.A)) on = !on;

        double target = RoboConstants.FLYWHEEL_TARGET_RPM;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP))    target += 100;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))  target -= 100;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) target += 25;
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))  target -= 25;
        RoboConstants.FLYWHEEL_TARGET_RPM = Math.max(0.0, Math.min(6000.0, target));

        if (on) flywheel.spinUp(); else flywheel.stop();
        flywheel.periodic(); // runs both PIDFs and adds RPM L/R + Power L/R telemetry

        tele.addData("Flywheel", on ? "ON" : "OFF (press A)");
        tele.addData("Target RPM", "%.0f", RoboConstants.FLYWHEEL_TARGET_RPM);
        tele.addData("Error L", "%.0f", RoboConstants.FLYWHEEL_TARGET_RPM - flywheel.getLeftRpm());
        tele.addData("Error R", "%.0f", RoboConstants.FLYWHEEL_TARGET_RPM - flywheel.getRightRpm());
        tele.addData("At speed (both)", flywheel.isAtSpeed());
        tele.addData("Left  kP/kI/kD/kF", "%.5f  %.5f  %.5f  %.6f", RoboConstants.FLYWHEEL_LEFT_kP,
                RoboConstants.FLYWHEEL_LEFT_kI, RoboConstants.FLYWHEEL_LEFT_kD, RoboConstants.FLYWHEEL_LEFT_kF);
        tele.addData("Right kP/kI/kD/kF", "%.5f  %.5f  %.5f  %.6f", RoboConstants.FLYWHEEL_RIGHT_kP,
                RoboConstants.FLYWHEEL_RIGHT_kI, RoboConstants.FLYWHEEL_RIGHT_kD, RoboConstants.FLYWHEEL_RIGHT_kF);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        flywheel.stop();
    }
}
