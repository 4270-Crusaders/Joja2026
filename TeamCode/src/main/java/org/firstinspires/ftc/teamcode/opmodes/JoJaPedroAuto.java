package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.utils.LoopTimer;

import java.util.ArrayList;
import java.util.List;

/**
 * Base for the Pedro Pathing 3 autos made in the Pedro Visualizer (files in Desktop/FTC BioBuzz/AutoPath).
 * 2026-11-12: rebuilt for Pedro 3 (the Pedro 2 version was removed 2026-11-09).
 *
 * Field: (0,0) = bottom-left corner, inches. Heading 0 = facing +X, counterclockwise +.
 * The paths are drawn for RED. BLUE = the same paths turned 180 deg around the field center.
 *
 * INIT: D-pad left = BLUE, D-pad right = RED (default RED). Keep the robot still during Init (Pinpoint calibrates).
 *
 * A subclass gives startPose() and buildSteps(). In buildSteps(), follow(...) drives one path
 * (it starts where the last one ended), and actions go between paths:
 *   spinUpFlywheel(s), shoot(s), intakeOn(), intakeOff(), matchLoaderDown/Middle/Up(s), waitFor(s)
 */
public abstract class JoJaPedroAuto extends OpMode {
    /** Field width used to flip RED -> BLUE. */
    private static final double FIELD_SIZE = 141.5;

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
    protected Interpolator linear(double startDeg, double endDeg) {
        return Interpolator.linear(heading(startDeg), heading(endDeg));
    }
    /** Same, but turning the LONG way around. */
    protected Interpolator linearLongWay(double startDeg, double endDeg) {
        return Interpolator.longLinear(heading(startDeg), heading(endDeg));
    }
    protected Interpolator constant(double deg) { return Interpolator.constant(heading(deg)); }
    /** Robot front faces the way it drives. */
    protected Interpolator tangent() { return Interpolator.tangent; }

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
    private final List<String> stepNames = new ArrayList<>();
    private int stepIndex = 0;
    private boolean stepStarted = false;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private Pose pathEnd; // where the next follow() starts
    private LoopTimer loopTimer; // 5-10 ms loop, telemetry every 50 ms

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = Constants.create(hardwareMap);
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
        tele.addData("Place robot at", "x %.1f  y %.1f  heading %.0f deg", s.x(), s.y(), Math.toDegrees(s.heading()));
        tele.addLine("Keep the robot still until Play.");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        // Paths are built here, after the alliance is picked
        Pose s = startPose();
        follower.setPose(s);
        pathEnd = s;
        buildSteps();
        loopTimer = new LoopTimer();
    }

    @Override
    public void loop() {
        loopTimer.tick();
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

        if (!LoopTimer.telemetryNow()) return;
        Pose p = follower.pose();
        tele.addData("Alliance", blue ? "BLUE" : "RED");
        tele.addData("Step", stepIndex < steps.size()
                ? (stepIndex + 1) + " / " + steps.size() + ": " + stepNames.get(stepIndex) : "DONE");
        tele.addData("Pose", "x %.1f  y %.1f  heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
        tele.addData("Loop avg / max ms", "%.1f / %.1f", loopTimer.getAverageMs(), loopTimer.getMaxMs());
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        LoopTimer.release();
        follower.stop();
        flywheel.stop();
        intake.setState(Intake.State.OFF);
        indexer.setState(Indexer.State.OFF);
    }

    private void add(String name, Step step) {
        stepNames.add(name);
        steps.add(step);
    }

    // ===== Step helpers =====

    /**
     * Drive one path from where the last path ended. points = control points (if any), then the END point.
     * Waits until the robot reaches the end of the path, then keeps holding that point during the next steps.
     */
    protected void follow(Interpolator heading, Pose... points) {
        Pose[] all = new Pose[points.length + 1];
        all[0] = pathEnd;
        System.arraycopy(points, 0, all, 1, points.length);
        final Path path = (all.length == 2 ? Paths.line(all[0], all[1]) : Paths.curve(all)).heading(heading);
        pathEnd = points[points.length - 1];
        final int number = countPaths() + 1;
        add("path " + number, new Step() {
            public void start() { follower.follow(path); } // Pedro holds the end point after (holdEnd)
            public boolean isDone() { return follower.atParametricEnd() || !follower.isBusy(); }
        });
    }

    private int countPaths() {
        int n = 0;
        for (String s : stepNames) if (s.startsWith("path")) n++;
        return n;
    }

    protected void waitFor(final double seconds) {
        add(String.format("wait %.1f s", seconds), new Step() {
            public void start() { }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    /** Flywheel on (RoboConstants.FLYWHEEL_TARGET_RPM), wait {@code seconds}. Keeps spinning until shoot() ends. */
    protected void spinUpFlywheel(final double seconds) {
        add("spin up flywheel", new Step() {
            public void start() { flywheel.spinUp(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    /**
     * Stopper open + intake + indexer feed for {@code seconds} (same proportional feed as JoJa Tele);
     * then flywheel off, stopper closed, feed off. Spins the flywheel up first if it isn't yet.
     */
    protected void shoot(final double seconds) {
        add(String.format("shoot %.1f s", seconds), new Step() {
            public void start() {
                flywheel.spinUp();
                stopper.open();
                intake.setState(Intake.State.SHOOTING); // INTAKE_SHOOT_POWER while shooting
            }
            public boolean isDone() {
                indexer.setState(flywheel.hasReachedSpeed() ? Indexer.State.FEEDING : Indexer.State.OFF);
                indexer.setFeedScale(flywheel.getFeedScale());
                intake.setShootScale(flywheel.getFeedScale());
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
        add("intake on", new Step() {
            public void start() {
                stopper.close();
                intake.setState(Intake.State.INTAKING);
                indexer.setState(Indexer.State.INTAKING);
            }
            public boolean isDone() { return true; }
        });
    }

    protected void intakeOff() {
        add("intake off", new Step() {
            public void start() {
                intake.setState(Intake.State.OFF);
                indexer.setState(Indexer.State.OFF);
            }
            public boolean isDone() { return true; }
        });
    }

    protected void matchLoaderDown(final double seconds) {
        add("match loader down", new Step() {
            public void start() { matchLoader.down(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    protected void matchLoaderMiddle(final double seconds) {
        add("match loader middle", new Step() {
            public void start() { matchLoader.middle(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    protected void matchLoaderUp(final double seconds) {
        add("match loader up", new Step() {
            public void start() { matchLoader.up(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }
}
