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
import org.firstinspires.ftc.teamcode.subsystems.AutoShoot;
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

    /** Where the robot ended (Pedro field), so JoJa Tele starts from it (hive aiming needs the real position). */
    public static Pose lastPose = null;

    protected abstract Pose startPose();   // RED coordinates, use pose(x, y, deg)
    protected abstract void buildSteps();

    // ===== Coordinates (always write RED numbers; these flip them for BLUE) =====

    private boolean blue = false;

    /**
     * 2026-11-13: JoJa's shooter is on the BACK. The visualizer headings are drawn as the way the SHOOTER faces,
     * so the robot's front (Pedro heading) is turned 180 deg from every drawn heading, and "tangential" paths
     * drive backward (shooter first). false = use the drawn headings as the robot's front.
     */
    public static boolean SHOOTER_ON_BACK = true;

    protected Pose pose(double x, double y, double headingDeg) {
        return blue ? new Pose(FIELD_SIZE - x, FIELD_SIZE - y, heading(headingDeg))
                    : new Pose(x, y, heading(headingDeg));
    }
    protected Pose pt(double x, double y) { return pose(x, y, 0); }
    /** Drawn heading (deg) -> robot front heading (rad): +180 for BLUE (field flip), +180 for the back shooter. */
    private double heading(double deg) {
        if (blue) deg += 180;
        if (SHOOTER_ON_BACK) deg += 180;
        return Math.toRadians(deg);
    }

    /** Turn from startDeg to endDeg along the path (the short way). */
    protected Interpolator linear(double startDeg, double endDeg) {
        return Interpolator.linear(heading(startDeg), heading(endDeg));
    }
    /** Same, but turning the LONG way around. */
    protected Interpolator linearLongWay(double startDeg, double endDeg) {
        return Interpolator.longLinear(heading(startDeg), heading(endDeg));
    }
    protected Interpolator constant(double deg) { return Interpolator.constant(heading(deg)); }
    /** The drawn front faces the way it drives (with SHOOTER_ON_BACK: the robot drives backward, shooter first). */
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
    protected AutoShoot autoShoot; // same automatic shooting as JoJa Tele (opens at target RPM, closes when empty)
    private List<LynxModule> hubs;

    private final List<Step> steps = new ArrayList<>();
    private final List<String> stepNames = new ArrayList<>();
    private int stepIndex = 0;
    private boolean stepStarted = false;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private Pose pathEnd; // where the next follow() starts
    private boolean loaderUpOnNextPath = false; // set by Flower / Garden: the next path raises the match loader
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
        autoShoot = new AutoShoot(flywheel, stopper, tele);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left) blue = true;
        if (gamepad1.dpad_right) blue = false;
        Pose s = startPose();
        tele.addData("Alliance (D-pad L = BLUE, R = RED)", blue ? "BLUE" : "RED");
        tele.addData("Place robot at", "x %.1f  y %.1f", s.x(), s.y());
        double frontDeg = ((Math.toDegrees(s.heading()) % 360) + 360) % 360;
        tele.addData("Robot FRONT (intake) faces", "%.0f deg%s", frontDeg,
                SHOOTER_ON_BACK ? String.format("   (shooter faces %.0f deg)", (frontDeg + 180) % 360) : "");
        tele.addLine("Keep the robot still until Play.");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        // Paths are built here, after the alliance is picked
        Pose s = startPose();
        follower.setPose(s);
        pathEnd = s;
        loaderUpOnNextPath = false;
        buildSteps();
        if (loaderUpOnNextPath) { // the auto ENDS with a Flower / Garden: still raise the match loader
            add("match loader up", new Step() {
                public void start() { matchLoader.up(); }
                public boolean isDone() { return true; }
            });
        }
        loopTimer = new LoopTimer();
    }

    @Override
    public void loop() {
        loopTimer.tick();
        for (LynxModule hub : hubs) hub.clearBulkCache();
        follower.update();
        lastPose = follower.pose();

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
        tele.addData("Flywheel L / R / target RPM", "%.0f / %.0f / %.0f",
                flywheel.getLeftRpm(), flywheel.getRightRpm(), flywheel.getTargetRpm());
        tele.addData("Last shot", lastShot);
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

    /** What to do AFTER a path (the path's name in the Pedro Visualizer). */
    protected enum Then {
        SHOOT,    // "Shoot..."   flywheel spins up DURING the path, then auto shoot at the end (like JoJa Tele)
        FLOWER,   // "Flower" (1.5tips-UP): drive to the end, THEN match loader DOWN + intake + indexer for
                  //               AUTO_FLOWER_WAIT_S (3 s), then off. The next path raises the match loader UP as it drives.
        COLLECT,  // "Garden": intake runs DURING the path, match loader DOWN AUTO_FLOWER_EARLY_S before the end,
                  //               stay AUTO_COLLECT_DWELL_S, then the next path raises the match loader UP as it
                  //               drives away (intake stays on, stopper closed, until the next shoot)
        INTAKE,   // drive with intake + indexer on (stopper closed), no stop, no match loader
        LOADER_DOWN, // "Matchloader-down": match loader DOWN BEFORE the path (waits AUTO_LOADER_MOVE_S for the
                     //               servo), drive, then stay there AUTO_FLOWER_WAIT_S (4 s) with intake + indexer on
        LOADER_UP,   // "Matchloader-up": drive, then match loader UP after the path
        PARK,     // "Park": everything off, stay there
        NOTHING   // no name: just drive
    }

    /** Drive one path, then do {@code then}. points = control points (if any), then the END point. */
    protected void path(String name, Then then, Interpolator heading, Pose... points) {
        switch (then) {
            case SHOOT:
                follow(name, new Runnable() { public void run() { flywheel.spinUp(); } }, null, heading, points);
                shootNow();
                break;
            case FLOWER:
                // 2026-11-14: drive to the end, THEN match loader down + intake + indexer 3 s (then off).
                // The next path raises the match loader while it drives.
                follow(name, null, null, heading, points);
                loaderDownStep();
                intakeForStep(RoboConstants.AUTO_FLOWER_WAIT_S);
                loaderUpOnNextPath = true;
                break;
            case COLLECT:
                follow(name, new Runnable() { public void run() { collectOn(); } },
                        new Runnable() { public void run() { matchLoader.down(); } }, heading, points);
                waitFor(RoboConstants.AUTO_COLLECT_DWELL_S);
                loaderUpOnNextPath = true; // match loader goes back up while driving the next path
                break;
            case INTAKE:
                follow(name, new Runnable() { public void run() { collectOn(); } }, null, heading, points);
                break;
            case LOADER_DOWN:
                // 1) match loader down (wait for the servo)  2) drive  3) intake + indexer for 3 s there, then off
                loaderDownStep();
                follow(name, null, null, heading, points);
                intakeForStep(RoboConstants.AUTO_FLOWER_WAIT_S);
                break;
            case LOADER_UP:
                follow(name, null, null, heading, points);
                add("match loader up", new Step() {
                    public void start() { matchLoader.up(); }
                    public boolean isDone() { return true; }
                });
                break;
            case PARK:
                follow(name, new Runnable() { public void run() { allOff(); } }, null, heading, points);
                break;
            default:
                follow(name, null, null, heading, points);
                break;
        }
    }

    /** Drive one path (no action). */
    protected void follow(Interpolator heading, Pose... points) { follow(null, null, null, heading, points); }

    /**
     * Drive one path from where the last path ended. points = control points (if any), then the END point.
     * onStart runs when the path starts (e.g. spin up the flywheel while driving). onNearEnd runs once, about
     * AUTO_FLOWER_EARLY_S before the robot reaches the end (from its speed and the distance left; at the latest
     * when the path ends). Waits until the robot reaches the end, then keeps holding that point.
     */
    private void follow(String name, final Runnable onStart, final Runnable onNearEnd, Interpolator heading,
                        Pose... points) {
        Pose[] all = new Pose[points.length + 1];
        all[0] = pathEnd;
        System.arraycopy(points, 0, all, 1, points.length);
        Path p = all.length == 2 ? Paths.line(all[0], all[1]) : Paths.curve(all);
        // tangential + shooter on the back = drive backward (back / shooter faces the way it drives)
        final Path path = (heading == Interpolator.tangent && SHOOTER_ON_BACK) ? p.reverseTangent() : p.heading(heading);
        pathEnd = points[points.length - 1];
        final int number = countPaths() + 1;
        final boolean loaderUp = loaderUpOnNextPath; // the path before was a Flower / Garden
        loaderUpOnNextPath = false;
        add("path " + number + (name != null && !name.isEmpty() ? " (" + name + ")" : ""), new Step() {
            boolean nearEndDone = false;
            public void start() {
                nearEndDone = false;
                if (loaderUp) matchLoader.up(); // raised while this path drives away from the Flower / Garden
                if (onStart != null) onStart.run();
                follower.follow(path); // Pedro holds the end point after (holdEnd)
            }
            public boolean isDone() {
                boolean done = follower.atParametricEnd() || !follower.isBusy();
                if (onNearEnd != null && !nearEndDone) {
                    // time left = distance left / speed (speed floored so a slow robot still triggers near the end)
                    double speed = Math.max(Math.abs(follower.tangentialVelocity()), 5.0); // in/s
                    double secondsLeft = follower.remainingDistance() / speed;
                    if (done || (stepTimer.seconds() > 0.1 && secondsLeft <= RoboConstants.AUTO_FLOWER_EARLY_S)) {
                        onNearEnd.run();
                        nearEndDone = true;
                    }
                }
                return done;
            }
        });
    }

    /** Match loader down, then wait AUTO_LOADER_MOVE_S so the servo is down before the next step. */
    private void loaderDownStep() {
        add("match loader down", new Step() {
            public void start() { matchLoader.down(); }
            public boolean isDone() { return stepTimer.seconds() >= RoboConstants.AUTO_LOADER_MOVE_S; }
        });
    }

    /** Stay here with intake + indexer on (stopper closed) for {@code seconds}, then turn them off. */
    private void intakeForStep(final double seconds) {
        add(String.format("intake %.1f s", seconds), new Step() {
            public void start() { collectOn(); }
            public boolean isDone() {
                if (stepTimer.seconds() < seconds) return false;
                intake.setState(Intake.State.OFF);
                indexer.setState(Indexer.State.OFF);
                return true;
            }
        });
    }

    private void collectOn() {
        stopper.close();
        intake.setState(Intake.State.INTAKING);
        indexer.setState(Indexer.State.INTAKING);
    }

    private void allOff() {
        flywheel.stop();
        stopper.close();
        intake.setState(Intake.State.OFF);
        indexer.setState(Indexer.State.OFF);
    }

    /**
     * Shoot like JoJa Tele with RT held: flywheel to target, the stopper opens once both sides are at target RPM
     * (AUTO_SHOOT_READY_...), indexer + intake feed (proportional feed), and the stopper closes once no more balls
     * come through (AUTO_SHOOT_EMPTY_S). Then everything off. Never longer than AUTO_SHOOT_STEP_MAX_S.
     */
    protected void shootNow() {
        add("shoot", new Step() {
            public void start() {
                flywheel.spinUp();
                autoShoot.update(false, true); // fresh start (IDLE)
            }
            public boolean isDone() {
                autoShoot.update(true, true);
                if (stopper.isOpen()) {
                    // feeding into the shooter (same as JoJa Tele while shooting)
                    indexer.setState(flywheel.hasReachedSpeed() ? Indexer.State.FEEDING : Indexer.State.OFF);
                    intake.setState(Intake.State.SHOOTING);
                } else {
                    // waiting for RPM: push the balls up against the closed stopper so they're ready
                    indexer.setState(Indexer.State.INTAKING);
                    intake.setState(Intake.State.INTAKING);
                }
                indexer.setFeedScale(flywheel.getFeedScale());
                intake.setShootScale(flywheel.getFeedScale());

                boolean done = autoShoot.getPhase() == AutoShoot.Phase.DONE;
                boolean timeout = stepTimer.seconds() >= RoboConstants.AUTO_SHOOT_STEP_MAX_S;
                if (!done && !timeout) return false;
                lastShot = done ? autoShoot.getBalls() + " balls, " + autoShoot.getLastReason()
                        : "TIMEOUT (never reached target RPM?)";
                autoShoot.update(false, true);
                allOff();
                return true;
            }
        });
    }
    private String lastShot = "-";

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
