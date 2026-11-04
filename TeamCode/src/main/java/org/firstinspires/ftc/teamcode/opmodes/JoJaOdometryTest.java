package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Pinpoint + odometry pods check. No motors run: push the robot by hand.
 *
 *   Push FORWARD -> X goes UP.     Push LEFT -> Y goes UP.     Turn LEFT (counterclockwise) -> heading goes UP.
 *
 * A          = reset X, Y, heading to 0 (keep the robot still for ~0.5 s while the IMU calibrates)
 * D-pad left = flip X pod direction    D-pad right = flip Y pod direction
 * X          = switch pod type (4-Bar / Swingarm)
 * Copy the final settings shown on screen into RoboConstants (PINPOINT_...).
 */
@TeleOp(name="JoJa Odometry Test", group="Test")
public class JoJaOdometryTest extends OpMode {
    private TelemetryManager panels;
    private Telemetry tele;
    private GoBildaPinpointDriver pinpoint;
    private boolean lastA, lastX, lastLeft, lastRight;

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RoboConstants.PINPOINT_NAME);
        applySettings();
        pinpoint.resetPosAndIMU();
        tele.addLine("JoJa Odometry Test. Keep the robot still, then press Play.");
        panels.update(telemetry);
    }

    private void applySettings() {
        pinpoint.setOffsets(RoboConstants.PINPOINT_X_OFFSET_IN, RoboConstants.PINPOINT_Y_OFFSET_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(RoboConstants.PINPOINT_4_BAR_POD
                ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                RoboConstants.PINPOINT_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                RoboConstants.PINPOINT_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    @Override
    public void loop() {
        pinpoint.update();

        // Button presses (only on the press, not while held)
        boolean changed = false;
        if (gamepad1.dpad_left && !lastLeft)   { RoboConstants.PINPOINT_X_REVERSED = !RoboConstants.PINPOINT_X_REVERSED; changed = true; }
        if (gamepad1.dpad_right && !lastRight) { RoboConstants.PINPOINT_Y_REVERSED = !RoboConstants.PINPOINT_Y_REVERSED; changed = true; }
        if (gamepad1.x && !lastX)              { RoboConstants.PINPOINT_4_BAR_POD = !RoboConstants.PINPOINT_4_BAR_POD; changed = true; }
        if (changed) applySettings();
        if ((gamepad1.a && !lastA) || changed) pinpoint.resetPosAndIMU();
        lastA = gamepad1.a; lastX = gamepad1.x; lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;

        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double h = pinpoint.getHeading(AngleUnit.DEGREES);

        tele.addData("Status", pinpoint.getDeviceStatus());
        tele.addData("X (forward +)", "%.2f in", x);
        tele.addData("Y (left +)", "%.2f in", y);
        tele.addData("Heading (CCW +)", "%.1f deg", h);
        tele.addLine("");
        tele.addData("Raw encoder X / Y", "%d / %d ticks", pinpoint.getEncoderX(), pinpoint.getEncoderY());
        tele.addData("Velocity X / Y", "%.1f / %.1f in/s", pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH));
        tele.addData("Pinpoint rate", "%.0f Hz", pinpoint.getFrequency());
        tele.addLine("");
        tele.addData("Pod type (X)", RoboConstants.PINPOINT_4_BAR_POD ? "4-Bar" : "Swingarm");
        tele.addData("X pod (D-pad L)", RoboConstants.PINPOINT_X_REVERSED ? "REVERSED" : "FORWARD");
        tele.addData("Y pod (D-pad R)", RoboConstants.PINPOINT_Y_REVERSED ? "REVERSED" : "FORWARD");
        tele.addLine("A = reset to 0. Push forward: X up. Push left: Y up. Turn left: heading up.");

        // Panels graph
        panels.addData("Graph/X in", Math.round(x * 100) / 100.0);
        panels.addData("Graph/Y in", Math.round(y * 100) / 100.0);
        panels.addData("Graph/Heading deg", Math.round(h * 10) / 10.0);
        panels.update(telemetry);
    }
}
