package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

import java.util.ArrayList;
import java.util.List;

/**
 * Base for the Pedro autos made in the Pedro Visualizer (files in Desktop/FTC BioBuzz/AutoPath).
 *
 * Field: (0,0) = bottom-left corner, inches. Heading 0 = facing +X, counterclockwise +.
 * The paths are drawn for RED. BLUE = the same paths turned 180 deg around the field center
 * (the BioBuzz field is symmetric that way: red FLOWER (0,47) <-> blue FLOWER (141.5,94.75)).
 *
 * INIT: D-pad left = BLUE, D-pad right = RED (default RED).
 *
 * A subclass gives startPose() and buildSteps(). In buildSteps(), follow(...) drives one path
 * (it starts where the last one ended), and actions go between paths:
 *   spinUpFlywheel(s), shoot(s), intakeOn(), intakeOff(), matchLoaderDown/Middle/Up(s), waitFor(s)
 */
public abstract class JoJaPedroAuto extends OpMode {
    /** Field width used to flip RED -> BLUE (from the FLOWER positions in the visualizer). */
    private static final double FIELD_SIZE = 141.5;

    /** Where the robot ended, so JoJa Tele (field centric) can start from it. */
    public static Pose lastPose = null;

    protected abstract Pose startPose();   // RED coordinates, use pose(x, y, deg)
    protected abstract void buildSteps();

    // ===== Coordinates (always write RED numbers; these flip them for BLUE) =====

    private boolean blue = false;

    protected Pose pose(double x, double y, double headingDeg) {
        return blue ? new Pose(FIELD_SIZE - x, FIELD_SIZE - y, Math.toRadians(headingDeg + 180))
                    : new Pose(x, y, Math.toRadians(headingDeg));
    }
    protected Pose pt(double x, double y) { return pose(x, y, 0); }
    private double heading(double deg) { return Math.toRadians(blue ? deg + 180 : deg); }

    /** Turn from startDeg to endDeg along the path (the short way). */
    protected HeadingInterpolator linear(double startDeg, double endDeg) {
        return HeadingInterpolator.linear(heading(startDeg), heading(endDeg));
    }
    /** Same, but turning the LONG way around (the visualizer's "reverse" on a linear heading). */
    protected HeadingInterpolator linearLongWay(double startDeg, double endDeg) {
        return HeadingInterpolator.reversedLinear(heading(startDeg), heading(endDeg));
    }
    protected HeadingInterpolator constant(double deg) { return HeadingInterpolator.constant(heading(deg)); }
    /** Robot front faces the way it drives. */
    protected HeadingInterpolator tangent() { return HeadingInterpolator.tangent; }
    /** Robot back faces the way it drives (drives backward). */
    protected HeadingInterpolator tangentReversed() { return HeadingInterpolator.tangent.reverse(); }

    // ===== Steps =====

    private interface Step {
        void start();
        boolean isDone();
    }

    private TelemetryManager panels;
    protected Telemetry tele;
    protected Follower follower;
    protected Flywheel flywheel;
    protected Intake intake;
    protected Indexer indexer;
    protected Stopper stopper;
    protected MatchLoader matchLoader;
    private List<LynxModule> hubs;

    private final List<Step> steps = new ArrayList<>();
    private int stepIndex = 0;
    private boolean stepStarted = false;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private Pose pathEnd; // where the next follow() starts

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = Constants.createFollower(hardwareMap, tele);
        Drawing.init();
        flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME, tele);
        intake = new Intake(hardwareMap, RoboConstants.INTAKE_MOTOR_NAME, tele);
        indexer = new Indexer(hardwareMap, RoboConstants.INDEXER_MOTOR_NAME, tele);
        stopper = new Stopper(hardwareMap, RoboConstants.STOPPER_SERVO_NAME, tele);             // starts CLOSED
        matchLoader = new MatchLoader(hardwareMap, RoboConstants.MATCH_LOADER_SERVO_NAME, tele); // starts UP
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left) blue = true;
        if (gamepad1.dpad_right) blue = false;
        Pose s = startPose();
        tele.addData("Alliance (D-pad L = BLUE, R = RED)", blue ? "BLUE" : "RED");
        tele.addData("Place robot at", "x %.1f  y %.1f  heading %.0f deg", s.getX(), s.getY(), Math.toDegrees(s.getHeading()));
        panels.update(telemetry);
    }

    @Override
    public void start() {
        // Paths are built here, after the alliance is picked
        Pose s = startPose();
        follower.setStartingPose(s);
        pathEnd = s;
        buildSteps();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        follower.update();

        if (stepIndex < steps.size()) {
            Step s = steps.get(stepIndex);
            if (!stepStarted) {
                stepTimer.reset();
                s.start();
                stepStarted = true;
            }
            if (s.isDone()) {
                stepIndex++;
                stepStarted = false;
            }
        }

        flywheel.setShooting(stopper.isOpen());
        flywheel.periodic();
        intake.periodic();
        indexer.periodic();
        stopper.periodic();
        matchLoader.periodic();

        Pose p = follower.getPose();
        lastPose = p;
        tele.addData("Step", stepIndex < steps.size() ? (stepIndex + 1) + " / " + steps.size() : "DONE");
        tele.addData("Pose", "x %.1f  y %.1f  heading %.0f deg", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        Drawing.drawDebug(follower);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        flywheel.stop();
        intake.setState(Intake.State.OFF);
        indexer.setState(Indexer.State.OFF);
    }

    // ===== Step helpers =====

    /**
     * Drive one path from where the last path ended. points = control points (if any), then the END point.
     * Waits until the robot gets there.
     */
    protected void follow(HeadingInterpolator heading, Pose... points) {
        List<Pose> all = new ArrayList<>();
        all.add(pathEnd);
        for (Pose p : points) all.add(p);
        final PathChain chain = follower.pathBuilder()
                .addPath(all.size() == 2 ? new BezierLine(all.get(0), all.get(1)) : new BezierCurve(all))
                .setHeadingInterpolation(heading)
                .build();
        pathEnd = points[points.length - 1];
        steps.add(new Step() {
            public void start() { follower.followPath(chain, true); } // true = hold the end point
            public boolean isDone() { return !follower.isBusy(); }
        });
    }

    protected void waitFor(final double seconds) {
        steps.add(new Step() {
            public void start() { }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    /** Flywheel on (RoboConstants.FLYWHEEL_TARGET_RPM), wait {@code seconds}. Keeps spinning until shoot() ends. */
    protected void spinUpFlywheel(final double seconds) {
        steps.add(new Step() {
            public void start() { flywheel.spinUp(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    /**
     * Stopper open + intake + indexer feed for {@code seconds} (indexer pauses while the flywheel recovers,
     * same as JoJa Tele); then flywheel off, stopper closed, feed off.
     */
    protected void shoot(final double seconds) {
        steps.add(new Step() {
            public void start() {
                flywheel.spinUp();
                stopper.open();
                intake.setState(Intake.State.INTAKING);
            }
            public boolean isDone() {
                indexer.setState(flywheel.readyToFeed() ? Indexer.State.FEEDING : Indexer.State.OFF);
                if (stepTimer.seconds() < seconds) return false;
                flywheel.stop();
                stopper.close();
                intake.setState(Intake.State.OFF);
                indexer.setState(Indexer.State.OFF);
                return true;
            }
        });
    }

    /** Intake + indexer on (collect speed), stopper closed so balls stay in. Instant (keeps running while driving). */
    protected void intakeOn() {
        steps.add(new Step() {
            public void start() {
                stopper.close();
                intake.setState(Intake.State.INTAKING);
                indexer.setState(Indexer.State.INTAKING);
            }
            public boolean isDone() { return true; }
        });
    }

    protected void intakeOff() {
        steps.add(new Step() {
            public void start() {
                intake.setState(Intake.State.OFF);
                indexer.setState(Indexer.State.OFF);
            }
            public boolean isDone() { return true; }
        });
    }

    protected void matchLoaderDown(final double seconds) {
        steps.add(new Step() {
            public void start() { matchLoader.down(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    protected void matchLoaderMiddle(final double seconds) {
        steps.add(new Step() {
            public void start() { matchLoader.middle(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    protected void matchLoaderUp(final double seconds) {
        steps.add(new Step() {
            public void start() { matchLoader.up(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }
}
