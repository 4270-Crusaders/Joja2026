package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

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

    // ===== Utils =====
    private GamepadEx driverOp;
    private boolean intakeWasHeld = false; // for left-trigger press / release edges
    private final ElapsedTime loopTimer = new ElapsedTime();
    private static final double FLYWHEEL_RUN_ON_SECONDS = 0.5; // flywheel keeps spinning this long after RT is released
    private final ElapsedTime sinceSpinReleased = new ElapsedTime();
    private boolean spunSinceStart = false;
    private java.util.List<LynxModule> hubs;


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

        // util
        driverOp = new GamepadEx(gamepad1);

        tele.addData(">", "Robot Ready.  Press Play.");
        panels.update(telemetry);
        loopTimer.reset();
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
        for (LynxModule hub : hubs) hub.clearBulkCache(); // fresh sensor readings this loop
        driverOp.readButtons();

        /*
         * Drive Stuff
         */
        drivetrain.drive(driverOp.getLeftY(), driverOp.getLeftX(), driverOp.getRightX());

        // ===== Match loader toggle =====
        if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) { // L1: match loader up/down
            matchLoader.toggle();
        }
        if (driverOp.wasJustPressed(GamepadKeys.Button.Y)) {   // Y: MIDDLE <-> UP
            if (matchLoader.isMiddle()) matchLoader.up(); else matchLoader.middle();
        }
        if (driverOp.wasJustPressed(GamepadKeys.Button.A)) {   // A: MIDDLE <-> DOWN
            if (matchLoader.isMiddle()) matchLoader.down(); else matchLoader.middle();
        }

        // Outtake (reverse intake + indexer): hold B or X
        boolean reverseHeld = driverOp.getButton(GamepadKeys.Button.B) || driverOp.getButton(GamepadKeys.Button.X);
        boolean spinHeld = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
        boolean intakeHeld = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD;

        // ===== Manual shooting: the driver decides when to shoot =====
        // Right trigger (hold) spins the flywheel ONLY (no intake). It keeps spinning FLYWHEEL_RUN_ON_SECONDS
        // after RT is released. Right bumper toggles the stopper OPEN / CLOSED.
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
        if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (stopper.isOpen()) stopper.close(); else stopper.open();
        }
        // Intaking closes the stopper automatically (on left-trigger press) so balls collect and hold.
        // Not while the flywheel is on: then the left trigger is how you feed balls into the shooter.
        if (intakeHeld && !intakeWasHeld && !flywheelOn) stopper.close();
        intakeWasHeld = intakeHeld;

        // Shooting = stopper open: the flywheel fights speed drops from POLLEN hitting the wheels at full power
        flywheel.setShooting(stopper.isOpen());
        // Feed gate: while a side is still recovering from the last ball, the indexer pauses so the next ball
        // doesn't hit an already-slow wheel (that's what took it down to ~2800)
        Indexer.State shootFeed = flywheel.readyToFeed() ? Indexer.State.FEEDING : Indexer.State.OFF;

        // ===== Intake / indexer: only the left trigger runs them (Reverse > Intake > idle) =====
        if (reverseHeld) {
            intake.setState(Intake.State.REVERSING);
            indexer.setState(Indexer.State.REVERSING);
        } else if (intakeHeld) {
            // Left trigger: intake + indexer. Flywheel on + stopper open = shooting feed speed, otherwise intake speed.
            intake.setState(Intake.State.INTAKING);
            indexer.setState(flywheelOn && stopper.isOpen() ? shootFeed : Indexer.State.INTAKING);
        } else if (flywheelOn) {
            // Flywheel on: indexer runs too (feed speed if stopper open, else holds balls at the stopper). Intake stays off.
            intake.setState(Intake.State.OFF);
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

        tele.addData("Flywheel (RT)", spinHeld ? "ON (held)" : flywheelOn
                ? String.format("ON (stops in %.1f s)", FLYWHEEL_RUN_ON_SECONDS - sinceSpinReleased.seconds()) : "OFF");
        tele.addData("Stopper (R1)", stopper.isOpen() ? "OPEN" : "CLOSED");
        tele.addData("Match loader (L1 up/down, Y mid/up, A mid/down)", matchLoader.getState());
        tele.addData("Loop", "%.1f ms", loopTimer.milliseconds());
        loopTimer.reset();
        graph();
        panels.update(telemetry);
    }

    /**
     * Panels graph: plain number values, so the Graph tab can plot them.
     * In Panels open Graph and pick "Graph/Target RPM", "Graph/Left RPM", "Graph/Right RPM".
     */
    private void graph() {
        panels.addData("Graph/Target RPM", Math.round(flywheel.getTargetRpm()));
        panels.addData("Graph/Left RPM", Math.round(flywheel.getLeftRpm()));
        panels.addData("Graph/Right RPM", Math.round(flywheel.getRightRpm()));
    }

    @Override

    public void stop() {
        drivetrain.stop();
        flywheel.stop();
        intake.setState(Intake.State.OFF);
        indexer.setState(Indexer.State.OFF);
    }
}
