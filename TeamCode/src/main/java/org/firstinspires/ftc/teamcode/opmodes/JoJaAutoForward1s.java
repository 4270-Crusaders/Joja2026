package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/** Simplest auto: drive straight forward for DRIVE_SECONDS at DRIVE_POWER, then stop. No Pedro needed. */
@Configurable
@Autonomous(name="JoJa Auto Forward 1s", group="Test")
public class JoJaAutoForward1s extends OpMode {
    public static double DRIVE_POWER = 0.5;   // 0..1, voltage-compensated to a 12 V battery
    public static double DRIVE_SECONDS = 1.0;

    private TelemetryManager panels;
    private Telemetry tele;
    private Drivetrain drivetrain;

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        drivetrain = new Drivetrain(hardwareMap,
                RoboConstants.DRIVE_LEFT_FRONT_NAME,
                RoboConstants.DRIVE_LEFT_BACK_NAME,
                RoboConstants.DRIVE_RIGHT_FRONT_NAME,
                RoboConstants.DRIVE_RIGHT_BACK_NAME,
                tele);

        tele.addData(">", "Forward %.1f s at %.2f power. Press Play.", DRIVE_SECONDS, DRIVE_POWER);
        panels.update(telemetry);
    }

    @Override
    public void start() {
        // forward, no strafe, no turn; Drivetrain stops itself when the time is up
        drivetrain.timedMove(DRIVE_POWER, 0.0, 0.0, DRIVE_SECONDS);
    }

    @Override
    public void loop() {
        drivetrain.periodic();
        tele.addData("Status", drivetrain.isBusy() ? "driving forward" : "done");
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}
