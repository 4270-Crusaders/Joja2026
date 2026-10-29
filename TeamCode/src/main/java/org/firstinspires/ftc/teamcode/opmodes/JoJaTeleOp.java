package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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


    @Override
    public void init() {
        // Panels: subsystems write to `tele`; panels.update(telemetry) sends it to Panels AND the Driver Hub
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
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

    @Override
    public void loop() {
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
        // Right trigger (hold) spins the flywheel; right bumper toggles the stopper OPEN / CLOSED.
        if (spinHeld) {
            flywheel.spinUp();
        } else {
            flywheel.stop();
        }
        if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (stopper.isOpen()) stopper.close(); else stopper.open();
        }
        // Intaking closes the stopper automatically (on left-trigger press) so balls collect and hold
        if (intakeHeld && !intakeWasHeld) stopper.close();
        intakeWasHeld = intakeHeld;

        // ===== Intake / indexer: Reverse > Intake or Spin > idle =====
        if (reverseHeld) {
            intake.setState(Intake.State.REVERSING);
            indexer.setState(Indexer.State.REVERSING);
        } else if (intakeHeld) {
            // Left trigger: intake + indexer at intake speed (stopper is closed automatically above)
            intake.setState(Intake.State.INTAKING);
            indexer.setState(Indexer.State.INTAKING);
        } else if (spinHeld) {
            // Right trigger (flywheel) also runs the intake roller + indexer: shooting feed if stopper open, else hold
            intake.setState(Intake.State.INTAKING);
            indexer.setState(stopper.isOpen() ? Indexer.State.FEEDING : Indexer.State.INTAKING);
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

        tele.addData("Stopper (R1)", stopper.isOpen() ? "OPEN" : "CLOSED");
        tele.addData("Match loader (L1 up/down, Y mid/up, A mid/down)", matchLoader.getState());
        tele.addData("Loop", "%.1f ms", loopTimer.milliseconds());
        loopTimer.reset();
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        drivetrain.stop();
        flywheel.stop();
        intake.setState(Intake.State.OFF);
        indexer.setState(Indexer.State.OFF);
    }
}
