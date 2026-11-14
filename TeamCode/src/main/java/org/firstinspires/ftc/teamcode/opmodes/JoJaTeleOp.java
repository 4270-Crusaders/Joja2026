package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.AutoShoot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.utils.Battery;
import org.firstinspires.ftc.teamcode.utils.CurrentMonitor;
import org.firstinspires.ftc.teamcode.utils.LoopTimer;

@TeleOp(name="JoJa Tele", group="Competition")
public class JoJaTeleOp extends OpMode {
    private TelemetryManager panels;
    private Telemetry tele;
    private static final double TRIGGER_THRESHOLD = 0.3;

    // ===== Subsystems =====
    private Drivetrain drivetrain;
    private Intake intake;
    private Indexer indexer;
    private Stopper stopper;
    private Flywheel flywheel;
    private MatchLoader matchLoader;
    private AutoShoot autoShoot;   // opens / closes the stopper by itself (RoboConstants.AUTO_SHOOT_...)
    // Pedro (Pinpoint) position on the Pedro field. In TeleOp it only reads the position (never drives the motors).
    private Follower follower;
    private boolean aiming = false;   // R1: turn so the shooter points at the nearest hive
    private String aimHiveName = "-";
    private double aimErrDeg = 0.0, aimDist = 0.0;

    // ===== Utils =====
    private GamepadEx driverOp;
    private boolean intakeWasHeld = false; // for left-trigger press / release edges
    private LoopTimer loopTimer; // 5-10 ms loop, telemetry every 50 ms (see utils/LoopTimer)
    private static final double FLYWHEEL_RUN_ON_SECONDS = 0.5; // flywheel keeps spinning this long after RT is released
    private final ElapsedTime sinceSpinReleased = new ElapsedTime();
    private boolean spunSinceStart = false;
    private java.util.List<LynxModule> hubs;
    private CurrentMonitor amps; // RoboConstants.CURRENT_MONITOR: amps per motor on the Driver Hub / Panels


    @Override
    public void init() {
        // Panels: subsystems write to `tele`; panels.update(telemetry) sends it to Panels AND the Driver Hub
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        // Bulk reads: all motor encoders/velocities in one read per hub per loop. Much faster loop, so the
        // flywheel reacts sooner when a POLLEN slows it. Cache is cleared at the start of every loop.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        drivetrain = new Drivetrain(hardwareMap,
                RoboConstants.DRIVE_LEFT_FRONT_NAME,
                RoboConstants.DRIVE_LEFT_BACK_NAME,
                RoboConstants.DRIVE_RIGHT_FRONT_NAME,
                RoboConstants.DRIVE_RIGHT_BACK_NAME, tele);
        intake = new Intake(hardwareMap, RoboConstants.INTAKE_MOTOR_NAME, tele);
        indexer = new Indexer(hardwareMap, RoboConstants.INDEXER_MOTOR_NAME, tele);
        stopper = new Stopper(hardwareMap, RoboConstants.STOPPER_SERVO_NAME, tele);
        flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME, tele);
        matchLoader = new MatchLoader(hardwareMap, RoboConstants.MATCH_LOADER_SERVO_NAME, tele);
        autoShoot = new AutoShoot(flywheel, stopper, tele);
        follower = Constants.create(hardwareMap); // keep the robot still during init (Pinpoint IMU calibrates)
        // Pedro just set the drive motors to FLOAT (coast), so the robot slid in TeleOp. Set BRAKE THROUGH Pedro:
        // it remembers the last setting and skips re-sending it. Setting it behind Pedro's back (2026-11-11 first fix)
        // made it skip switching to FLOAT for paths, so R1 drove with brakes on and stopped in the wrong spot.
        ((Mecanum) follower.drivetrain).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Start where the auto ended (real field position, needed for hive aiming); no auto run = the A pose
        follower.setPose(startPose());

        // util
        driverOp = new GamepadEx(gamepad1);
        amps = new CurrentMonitor()
                .add(hardwareMap, "Flywheel L", RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME)
                .add(hardwareMap, "Flywheel R", RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME)
                .add(hardwareMap, "Indexer", RoboConstants.INDEXER_MOTOR_NAME)
                .add(hardwareMap, "Intake", RoboConstants.INTAKE_MOTOR_NAME)
                .add(hardwareMap, "Drive LF", RoboConstants.DRIVE_LEFT_FRONT_NAME)
                .add(hardwareMap, "Drive LB", RoboConstants.DRIVE_LEFT_BACK_NAME)
                .add(hardwareMap, "Drive RF", RoboConstants.DRIVE_RIGHT_FRONT_NAME)
                .add(hardwareMap, "Drive RB", RoboConstants.DRIVE_RIGHT_BACK_NAME);

        tele.addData(">", "Robot Ready.  Press Play.");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        loopTimer = new LoopTimer();
        follower.setPose(startPose()); // again at Play: the Pinpoint may still have been calibrating at Init
        // used once: the next TeleOp (without a new auto first) must not start from this old auto position
        startedFromAuto = JoJaPedroAuto.lastPose != null;
        JoJaPedroAuto.lastPose = null;
    }

    private boolean startedFromAuto = false;

    /** Where the auto ended, or the A pose if no auto ran since the last TeleOp. */
    private static Pose startPose() {
        return JoJaPedroAuto.lastPose != null ? JoJaPedroAuto.lastPose : aPose();
    }

    // During INIT too, so the Graph variables show up in Panels before you press Play
    @Override
    public void init_loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        tele.addData(">", "Robot Ready.  Press Play.");
        graph();
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        loopTimer.tick(); // steady loop time + decides if this is a telemetry loop
        for (LynxModule hub : hubs) hub.clearBulkCache(); // fresh sensor readings this loop (one read per hub)
        driverOp.readButtons();

        /*
         * Drive Stuff
         */
        double fwd = driverOp.getLeftY(), strafe = driverOp.getLeftX(), turn = driverOp.getRightX();
        follower.localizer.update(); // position only: in TeleOp Pedro never drives the motors

        // ===== R1: aim at the nearest hive (2026-11-13). You keep driving with the left stick; the robot turns
        // itself so the shooter points at the hive. R1 again, or moving the right stick, = you take over turning =====
        if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) aiming = !aiming;
        if (aiming && Math.abs(turn) > RoboConstants.AIM_CANCEL_STICK) aiming = false;
        if (aiming) turn = aimTurn();
        drivetrain.drive(fwd, strafe, turn);

        // ===== Match loader toggle =====
        if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) { // L1: match loader up/down
            matchLoader.toggle();
        }
        // 2026-11-09: the match loader MIDDLE position has no button (MatchLoader.middle() etc. still exist).

        // ===== Odometry: A = this spot is (0,0) (heading kept) =====
        if (driverOp.wasJustPressed(GamepadKeys.Button.A)) follower.setPose(aPose()); // x, y AND heading (RoboConstants A_...)

        // Outtake (reverse intake + indexer): hold X (2026-11-11: was B; B now does nothing)
        boolean reverseHeld = driverOp.getButton(GamepadKeys.Button.X);
        boolean spinHeld = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
        boolean intakeHeld = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD;

        // ===== Manual shooting: the driver decides when to shoot =====
        // Right trigger (hold) spins the flywheel ONLY (no intake). It keeps spinning FLYWHEEL_RUN_ON_SECONDS
        // after RT is released. Y toggles the stopper OPEN / CLOSED (2026-11-11: was R1).
        if (spinHeld) {
            spunSinceStart = true;
            sinceSpinReleased.reset();
        }
        boolean flywheelOn = spinHeld || (spunSinceStart && sinceSpinReleased.seconds() < FLYWHEEL_RUN_ON_SECONDS);
        if (flywheelOn) {
            flywheel.spinUp();
        } else {
            flywheel.stop();
        }
        if (driverOp.wasJustPressed(GamepadKeys.Button.Y)) {
            if (stopper.isOpen()) stopper.close(); else stopper.open();
            autoShoot.cancel(); // driver took over: no auto open / close until RT is released
        }
        // Auto shoot: RT held + both sides at target RPM -> stopper opens; balls gone -> stopper closes
        autoShoot.update(spinHeld);
        // 2026-11-09: RT not held -> stopper always closed (also after opening it by hand with Y)
        if (!spinHeld) stopper.close();
        // Intaking closes the stopper automatically (on left-trigger press) so balls collect and hold.
        // Not while the flywheel is on: then the left trigger is how you feed balls into the shooter.
        if (intakeHeld && !intakeWasHeld && !flywheelOn) stopper.close();
        intakeWasHeld = intakeHeld;

        // Shooting = stopper open: the flywheel fights speed drops from POLLEN hitting the wheels at full power
        flywheel.setShooting(stopper.isOpen());
        // Feed gate: while a side is still recovering from the last ball, the indexer pauses so the next ball
        // doesn't hit an already-slow wheel (that's what took it down to ~2800)
        // It slows down (FEEDING_SLOW) instead of stopping: restarting from a dead stop browned out the hub.
        // Spin-up (wheels haven't reached speed yet): indexer waits completely (OFF).
        // 2026-11-05 proportional feed: once at speed, the indexer + intake slow down in proportion to the
        // flywheel dip (feed scale 1 = full, 0 = minimum) instead of full / slow. No stop-restart delays, and
        // they take less battery exactly when the flywheel needs it.
        Indexer.State shootFeed = flywheel.hasReachedSpeed() ? Indexer.State.FEEDING : Indexer.State.OFF;
        boolean shootingNow = flywheelOn && stopper.isOpen();
        double feedScale = flywheel.getFeedScale();
        indexer.setFeedScale(feedScale);
        intake.setShootScale(feedScale);
        // Load shedding: while shooting, drive power drops toward DRIVE_SHOOT_MIN_SCALE as the dip gets worse
        // (1.0 = off). Spin-up isn't limited (feed scale is 0 then, but nobody is shooting yet).
        double driveScale = (shootingNow && flywheel.hasReachedSpeed())
                ? 1.0 - (1.0 - RoboConstants.DRIVE_SHOOT_MIN_SCALE) * (1.0 - feedScale) : 1.0;
        drivetrain.setPowerScale(driveScale);

        // ===== Intake / indexer: only the left trigger runs them (Reverse > Intake > idle) =====
        if (reverseHeld) {
            intake.setState(Intake.State.REVERSING);
            indexer.setState(Indexer.State.REVERSING);
        } else if (intakeHeld) {
            // Left trigger: intake + indexer. Flywheel on + stopper open = shooting feed speed, otherwise intake speed.
            intake.setState(shootingNow ? Intake.State.SHOOTING : Intake.State.INTAKING);
            indexer.setState(shootingNow ? shootFeed : Indexer.State.INTAKING);
        } else if (flywheelOn) {
            // Flywheel on: indexer runs too (feed speed if stopper open, else holds balls at the stopper).
            // Shooting (stopper open): intake runs at INTAKE_SHOOT_POWER to keep balls coming. Otherwise off.
            intake.setState(stopper.isOpen() ? Intake.State.SHOOTING : Intake.State.OFF);
            indexer.setState(stopper.isOpen() ? shootFeed : Indexer.State.INTAKING);
        } else {
            intake.setState(Intake.State.OFF);
            indexer.setState(Indexer.State.OFF);
        }

        // subsystem
        drivetrain.periodic();
        flywheel.periodic();
        intake.periodic();
        indexer.periodic();
        stopper.periodic();
        matchLoader.periodic();
        amps.update();

        if (!LoopTimer.telemetryNow()) return; // telemetry is slow: only every TELEMETRY_PERIOD_MS
        tele.addData("Battery", Battery.isLow() ? "%.2f V  LOW: swap battery soon" : "%.2f V", Battery.volts());
        tele.addData("Flywheel (RT)", spinHeld ? "ON (held)" : flywheelOn
                ? String.format("ON (stops in %.1f s)", FLYWHEEL_RUN_ON_SECONDS - sinceSpinReleased.seconds()) : "OFF");
        tele.addData("Stopper (Y)", stopper.isOpen() ? "OPEN" : "CLOSED");
        autoShoot.addTelemetry();
        Pose pose = follower.pose();
        tele.addData("Aim at hive (R1)", !aiming ? "OFF"
                : String.format("%s hive  %s  (error %.1f deg, %.0f in away)", aimHiveName,
                        Math.abs(aimErrDeg) <= RoboConstants.AIM_TOLERANCE_DEG ? "AIMED" : "turning", aimErrDeg, aimDist));
        tele.addData("Position from", startedFromAuto ? "end of auto (A = reset)" : "A pose (no auto ran)");
        tele.addData("Odometry/X in", "%.2f", pose.x());
        tele.addData("Odometry/Y in", "%.2f", pose.y());
        tele.addData("Odometry/Heading deg", "%.1f", Math.toDegrees(pose.heading()));
        tele.addData("Match loader (L1 up/down)", matchLoader.getState());
        tele.addData("Loop ms", "%.1f", loopTimer.getLoopMs());
        tele.addData("Loop avg ms", "%.1f", loopTimer.getAverageMs());
        tele.addData("Loop max ms (last 1 s)", "%.1f", loopTimer.getMaxMs());
        tele.addData("Drive power scale", "%.2f", driveScale);
        amps.addTelemetry(tele);
        graph();
        panels.update(telemetry);
    }

    /** The A button pose: RoboConstants A_X, A_Y, A_HEADING_DEG. */
    private static Pose aPose() {
        return new Pose(RoboConstants.A_X, RoboConstants.A_Y, Math.toRadians(RoboConstants.A_HEADING_DEG));
    }

    /**
     * R1 aim: turn power that points the SHOOTER at the nearest hive (Pedro field coordinates, RoboConstants HIVE_...).
     * The shooter is on the back (AIM_SHOOTER_ON_BACK), so the robot's front points AWAY from the hive.
     * Proportional turn: AIM_KP * error, at most AIM_MAX_TURN, at least AIM_MIN_TURN (so it doesn't stall just short),
     * 0 once within AIM_TOLERANCE_DEG.
     */
    private double aimTurn() {
        Pose p = follower.pose();
        double dBottom = Math.hypot(RoboConstants.HIVE_BOTTOM_X - p.x(), RoboConstants.HIVE_BOTTOM_Y - p.y());
        double dTop = Math.hypot(RoboConstants.HIVE_TOP_X - p.x(), RoboConstants.HIVE_TOP_Y - p.y());
        boolean bottom = dBottom <= dTop;
        double hx = bottom ? RoboConstants.HIVE_BOTTOM_X : RoboConstants.HIVE_TOP_X;
        double hy = bottom ? RoboConstants.HIVE_BOTTOM_Y : RoboConstants.HIVE_TOP_Y;
        aimHiveName = bottom ? "BOTTOM" : "TOP";
        aimDist = Math.min(dBottom, dTop);

        double toHive = Math.atan2(hy - p.y(), hx - p.x());                     // direction robot -> hive
        double want = toHive + (RoboConstants.AIM_SHOOTER_ON_BACK ? Math.PI : 0.0); // front heading we need
        double err = want - p.heading();
        err = Math.atan2(Math.sin(err), Math.cos(err)); // -180..180: always the short way (+ = turn left / CCW)
        aimErrDeg = Math.toDegrees(err);

        if (Math.abs(aimErrDeg) <= RoboConstants.AIM_TOLERANCE_DEG) return 0.0;
        double ccw = RoboConstants.AIM_KP * err;
        ccw = Math.signum(ccw) * Math.max(RoboConstants.AIM_MIN_TURN, Math.min(RoboConstants.AIM_MAX_TURN, Math.abs(ccw)));
        return -ccw; // Drivetrain turn + = clockwise
    }

    /**
     * Panels graph: plain number values, so the Graph tab can plot them.
     * In Panels open Graph and pick "Graph/Target RPM", "Graph/Left RPM", "Graph/Right RPM", "Graph/Top", "Graph/Bottom".
     *
     * Zoom: the Panels graph has no y-axis setting, it always fits whatever lines are ticked. So the RPM lines are
     * clamped to TARGET +/- GRAPH_RPM_WINDOW, and Top / Bottom are flat lines at the window edges. With all five
     * ticked, the y-axis stays fixed on the target window instead of 0 to max. A line sitting flat on Top or Bottom
     * means the real RPM is outside the window (e.g. at 0 during spin-up); the Driver Hub still shows the real value.
     */
    private void graph() {
        double target = flywheel.getTargetRpm();
        double lo = target - RoboConstants.GRAPH_RPM_WINDOW;
        double hi = target + RoboConstants.GRAPH_RPM_WINDOW;
        panels.addData("Graph/Target RPM", Math.round(target));
        panels.addData("Graph/Left RPM", Math.round(Math.max(lo, Math.min(hi, flywheel.getLeftRpm()))));
        panels.addData("Graph/Right RPM", Math.round(Math.max(lo, Math.min(hi, flywheel.getRightRpm()))));
        panels.addData("Graph/Top", Math.round(hi));
        panels.addData("Graph/Bottom", Math.round(lo));
    }

    @Override

    public void stop() {
        LoopTimer.release();
        drivetrain.stop();
        flywheel.stop();
        intake.setState(Intake.State.OFF);
        indexer.setState(Indexer.State.OFF);
    }
}
