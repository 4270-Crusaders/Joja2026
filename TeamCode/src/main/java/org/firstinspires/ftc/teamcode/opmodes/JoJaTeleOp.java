package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

@TeleOp(name="JoJa Tele", group="Competition")
public class JoJaTeleOp extends OpMode {
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
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Set once the flywheel first reaches speed during a shoot hold; cleared on release or reverse
    private boolean shootLatched = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap,
                RoboConstants.DRIVE_LEFT_FRONT_NAME,
                RoboConstants.DRIVE_LEFT_BACK_NAME,
                RoboConstants.DRIVE_RIGHT_FRONT_NAME,
                RoboConstants.DRIVE_RIGHT_BACK_NAME,
                telemetry);
        intake = new Intake(hardwareMap, RoboConstants.INTAKE_MOTOR_NAME, telemetry);
        indexer = new Indexer(hardwareMap, RoboConstants.INDEXER_MOTOR_NAME, telemetry);
        stopper = new Stopper(hardwareMap, RoboConstants.STOPPER_SERVO_NAME, telemetry);
        flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME, telemetry);
        matchLoader = new MatchLoader(hardwareMap, RoboConstants.MATCH_LOADER_SERVO_NAME, telemetry);

        // util
        driverOp = new GamepadEx(gamepad1);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
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
        if (driverOp.wasJustPressed(GamepadKeys.Button.A)) {
            matchLoader.toggle();
        }

        boolean reverseHeld = driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean shootHeld = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
        boolean intakeHeld = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD;
        boolean spinHeld = driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);

        // ===== Flywheel: spin while Spin or Shoot held =====
        if (spinHeld || shootHeld) {
            flywheel.spinUp();
        } else {
            flywheel.stop();
        }

        // ===== Coordination: Reverse > Shoot > Intake / Match Loader UP > idle =====
        if (reverseHeld) {
            shootLatched = false;
            intake.setState(Intake.State.REVERSING);
            indexer.setState(Indexer.State.REVERSING);
            stopper.close();
        } else if (shootHeld) {
            if (flywheel.isAtSpeed()) {
                shootLatched = true;
            }
            if (shootLatched) {
                stopper.open();
                indexer.setState(Indexer.State.FEEDING);
                intake.setState(Intake.State.INTAKING);
            } else {
                stopper.close();
                indexer.setState(Indexer.State.OFF);
                intake.setState(Intake.State.OFF);
            }
        } else if (intakeHeld || matchLoader.isUp()) {
            shootLatched = false;
            intake.setState(Intake.State.INTAKING);
            indexer.setState(Indexer.State.INTAKING);
            stopper.close();
        } else {
            shootLatched = false;
            intake.setState(Intake.State.OFF);
            indexer.setState(Indexer.State.OFF);
            stopper.close();
        }

        // subsystem
        drivetrain.periodic();
        flywheel.periodic();
        intake.periodic();
        indexer.periodic();
        stopper.periodic();
        matchLoader.periodic();

        telemetry.addData("Shoot/Latched", shootLatched);
        telemetry.addData("Loop", "%.1f ms", loopTimer.milliseconds());
        loopTimer.reset();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        flywheel.stop();
        intake.setState(Intake.State.OFF);
        indexer.setState(Indexer.State.OFF);
    }
}
