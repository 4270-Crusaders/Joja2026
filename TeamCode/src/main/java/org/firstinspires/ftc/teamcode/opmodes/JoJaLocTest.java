package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import java.util.List;

/**
 * Localization test with the Pinpoint + odometry pods. Field: (0,0) = bottom-left corner, inches,
 * heading 0 = facing +X, counterclockwise +.
 *
 * Place the robot at START_X / START_Y / START_HEADING_DEG (edit here or live in Panels), then Play.
 * The robot does one move, then holds still. Screen + Panels field show where it thinks it is.
 * Measure where it REALLY ended with a tape measure and compare to "Pose now".
 * Also logged to logcat every 0.1 s, tag "[LocTest]".
 */
@Configurable
public abstract class JoJaLocTest extends OpMode {
    public static double START_X = 72.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_DEG = 90.0;
    public static double MOVE_INCHES = 36.0;     // 3 ft
    public static double TIMEOUT_SECONDS = 6.0;  // stops trying after this

    protected enum Move { FORWARD, STRAFE_RIGHT, ROTATE_180 }
    protected abstract Move move();

    private TelemetryManager panels;
    private Telemetry tele;
    private Follower follower;
    private List<LynxModule> hubs;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastLog = -1;
    private Pose start, target;
    private boolean done = false;

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = Constants.createPinpointFollower(hardwareMap);
        Drawing.init();
    }

    @Override
    public void init_loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        follower.update();
        tele.addData("Test", move());
        tele.addData("Place robot at", "x %.1f  y %.1f  heading %.0f deg", START_X, START_Y, START_HEADING_DEG);
        tele.addLine("Keep the robot still until Play.");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        double h = Math.toRadians(START_HEADING_DEG);
        start = new Pose(START_X, START_Y, h);
        follower.setStartingPose(start);

        switch (move()) {
            case FORWARD:       // along the way the robot faces
                target = new Pose(START_X + MOVE_INCHES * Math.cos(h), START_Y + MOVE_INCHES * Math.sin(h), h);
                break;
            case STRAFE_RIGHT:  // 90 deg clockwise from the way the robot faces
                target = new Pose(START_X + MOVE_INCHES * Math.cos(h - Math.PI / 2),
                                  START_Y + MOVE_INCHES * Math.sin(h - Math.PI / 2), h);
                break;
            default:            // turn in place, counterclockwise
                target = new Pose(START_X, START_Y, h + Math.PI);
                break;
        }

        if (move() == Move.ROTATE_180) {
            // 2026-11-05: NOT follower.turnDegrees(). It builds its target from getPose(), which right after
            // setStartingPose() is still the OLD pose (0,0, 0 deg): the robot turned only 90 deg and drove
            // diagonally toward (0,0). Hold the exact target built from the start pose instead.
            follower.holdPoint(target);
        } else {
            PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(start, target))
                    .setConstantHeadingInterpolation(h)
                    .build();
            follower.followPath(path, true); // hold the end point
        }
        timer.reset();
        RobotLog.i("[LocTest] %s start x=%.2f y=%.2f h=%.1f  target x=%.2f y=%.2f h=%.1f", move(),
                start.getX(), start.getY(), START_HEADING_DEG, target.getX(), target.getY(), Math.toDegrees(target.getHeading()));
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        follower.update();

        double t = timer.seconds();
        // Rotate: done once within 2 deg of the target heading (holdPoint never reports "busy")
        boolean arrived = (move() == Move.ROTATE_180)
                ? Math.abs(headingErrDeg(follower.getPose())) < 2.0
                : !follower.isBusy() && !follower.isTurning();
        if (!done && ((arrived && t > 0.3) || t > TIMEOUT_SECONDS)) {
            done = true;
            Pose p = follower.getPose();
            RobotLog.i("[LocTest] %s DONE in %.2f s  end x=%.2f y=%.2f h=%.1f  error x=%.2f y=%.2f h=%.1f%s", move(), t,
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading()),
                    p.getX() - target.getX(), p.getY() - target.getY(), headingErrDeg(p), t > TIMEOUT_SECONDS ? " (TIMEOUT)" : "");
        }

        Pose p = follower.getPose();
        if (t - lastLog >= 0.1) {
            lastLog = t;
            RobotLog.i("[LocTest] t=%.2f x=%.2f y=%.2f h=%.1f", t, p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        }

        tele.addData("Test", "%s   %s", move(), done ? "DONE (holding)" : String.format("moving %.1f s", t));
        tele.addData("Start ", "x %.2f  y %.2f  heading %.1f", start.getX(), start.getY(), START_HEADING_DEG);
        tele.addData("Target", "x %.2f  y %.2f  heading %.1f", target.getX(), target.getY(), norm(Math.toDegrees(target.getHeading())));
        tele.addData("Pose now", "x %.2f  y %.2f  heading %.1f", p.getX(), p.getY(), norm(Math.toDegrees(p.getHeading())));
        tele.addData("Error (now - target)", "x %.2f  y %.2f  heading %.1f", p.getX() - target.getX(), p.getY() - target.getY(), headingErrDeg(p));
        tele.addData("Moved", "%.2f in", Math.hypot(p.getX() - start.getX(), p.getY() - start.getY()));
        panels.addData("Graph/X", Math.round(p.getX() * 100) / 100.0);
        panels.addData("Graph/Y", Math.round(p.getY() * 100) / 100.0);
        panels.addData("Graph/Heading", Math.round(norm(Math.toDegrees(p.getHeading())) * 10) / 10.0);
        Drawing.drawDebug(follower);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }

    /** Degrees in 0..360. */
    private static double norm(double deg) { return ((deg % 360) + 360) % 360; }

    /** Heading error in -180..180 degrees. */
    private double headingErrDeg(Pose p) {
        double e = Math.toDegrees(p.getHeading() - target.getHeading());
        return ((e + 540) % 360) - 180;
    }
}
