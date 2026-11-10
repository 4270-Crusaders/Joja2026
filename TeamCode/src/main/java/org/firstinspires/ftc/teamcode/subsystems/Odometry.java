package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.utils.LoopTimer;

/**
 * TeleOp position tracking with the goBILDA Pinpoint + odometry pods (2026-11-09).
 * Same field as Pedro Pathing: inches, (0,0) = bottom-left corner, (144,144) = top-right,
 * heading in degrees, 0 = facing +X (right), counterclockwise +.
 *
 * Settings (pod type, directions, offsets) come from RoboConstants PINPOINT_..., same as the autos.
 * Later this is what auto-aim / "am I in a shooting zone" will use.
 */
public class Odometry {
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;
    private double x = 0.0, y = 0.0, headingDeg = 0.0;

    public Odometry(HardwareMap hMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        pinpoint = hMap.get(GoBildaPinpointDriver.class, RoboConstants.PINPOINT_NAME);
        pinpoint.setOffsets(RoboConstants.PINPOINT_X_OFFSET_IN, RoboConstants.PINPOINT_Y_OFFSET_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(RoboConstants.PINPOINT_4_BAR_POD
                ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                RoboConstants.PINPOINT_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                RoboConstants.PINPOINT_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        RobotLog.i("[Odometry] Initialized.");
    }

    /** Full reset: position (0,0), heading 0, and recalibrate the IMU. Robot must be still (~0.25 s). */
    public void resetAll() { pinpoint.resetPosAndIMU(); x = y = headingDeg = 0.0; }

    /** Set the position, keep the current heading. */
    public void setXY(double xIn, double yIn) {
        pinpoint.setPosX(xIn, DistanceUnit.INCH);
        pinpoint.setPosY(yIn, DistanceUnit.INCH);
        x = xIn;
        y = yIn;
    }

    /** Set position AND heading (e.g. the pose the auto ended at). */
    public void setPose(double xIn, double yIn, double headingDegrees) {
        setXY(xIn, yIn);
        pinpoint.setHeading(headingDegrees, AngleUnit.DEGREES);
        headingDeg = headingDegrees;
    }

    /** Read the Pinpoint. Call once per loop. */
    public void update() {
        pinpoint.update();
        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeadingDeg() { return headingDeg; }

    public void addTelemetry() {
        if (!LoopTimer.telemetryNow()) return;
        telemetry.addLine("====== Odometry ======");
        telemetry.addData("Odometry/X in", "%.2f", x);
        telemetry.addData("Odometry/Y in", "%.2f", y);
        telemetry.addData("Odometry/Heading deg", "%.1f", headingDeg);
    }
}
