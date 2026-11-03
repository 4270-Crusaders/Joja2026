package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Drivetrain motors only, one wheel at a time. Uses the same names and directions as JoJa Tele.
 *
 * Pick a wheel (press):
 *   Y = front left   (leftFront,  CH M1)
 *   X = back left    (leftBack,   CH M0)
 *   B = front right  (rightFront, EH M1)
 *   A = back right   (rightBack,  EH M0)
 * Left stick up/down = drive the picked wheel (up = should roll the robot FORWARD).
 * Only the picked wheel ever gets power.
 *
 * Put the robot on a stand first.
 */
@TeleOp(name="JoJa Drive Motor Test", group="Test")
public class JoJaDriveMotorTest extends OpMode {
    private static final String[] NAMES = {
            RoboConstants.DRIVE_LEFT_FRONT_NAME,
            RoboConstants.DRIVE_LEFT_BACK_NAME,
            RoboConstants.DRIVE_RIGHT_FRONT_NAME,
            RoboConstants.DRIVE_RIGHT_BACK_NAME
    };
    private static final String[] LABELS = {
            "Y  front left  (CH M1)",
            "X  back left   (CH M0)",
            "B  front right (EH M1)",
            "A  back right  (EH M0)"
    };
    private static final DcMotorSimple.Direction[] DIRECTIONS = {
            RoboConstants.DRIVE_LEFT_FRONT_DIRECTION,
            RoboConstants.DRIVE_LEFT_BACK_DIRECTION,
            RoboConstants.DRIVE_RIGHT_FRONT_DIRECTION,
            RoboConstants.DRIVE_RIGHT_BACK_DIRECTION
    };
    private static final int FL = 0, BL = 1, FR = 2, BR = 3;

    private TelemetryManager panels;
    private Telemetry tele;
    private GamepadEx driverOp;
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private int selected = -1; // none until a button is pressed

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        driverOp = new GamepadEx(gamepad1);

        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i]);
            motors[i].setDirection(DIRECTIONS[i]);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setPower(0.0);
        }

        tele.addLine("JoJa Drive Motor Test. Robot on a stand!");
        tele.addLine("Y=FL  X=BL  B=FR  A=BR, then left stick up/down.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        driverOp.readButtons();

        if (driverOp.wasJustPressed(GamepadKeys.Button.Y)) selected = FL;
        if (driverOp.wasJustPressed(GamepadKeys.Button.X)) selected = BL;
        if (driverOp.wasJustPressed(GamepadKeys.Button.B)) selected = FR;
        if (driverOp.wasJustPressed(GamepadKeys.Button.A)) selected = BR;

        double stick = driverOp.getLeftY(); // + = pushed up
        if (Math.abs(stick) < 0.05) stick = 0.0;

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(i == selected ? stick : 0.0);
        }

        tele.addData("Selected", selected < 0 ? "none (press Y / X / B / A)" : LABELS[selected]);
        tele.addData("Power", "%.2f  (stick up = wheel should roll robot FORWARD)", stick);
        tele.addLine("");
        for (int i = 0; i < 4; i++) {
            tele.addData(LABELS[i], "%s  pos %d  vel %.0f t/s  %s",
                    i == selected ? ">>" : "  ",
                    motors[i].getCurrentPosition(),
                    motors[i].getVelocity(),
                    NAMES[i]);
        }
        tele.addLine("");
        tele.addLine("Check: the RIGHT wheel turns, FORWARD with stick up, encoder pos goes UP.");
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        for (DcMotorEx m : motors) m.setPower(0.0);
    }
}
