package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

/**
 * Timed auto (no Pedro):
 *   1. strafe left      STRAFE_LEFT_SECONDS
 *   2. drive forward    FORWARD_SECONDS
 *   3. flywheel on at FLYWHEEL_RPM, wait SPINUP_SECONDS
 *   4. open stopper + feed (intake + indexer) for SHOOT_SECONDS, then flywheel off, stopper closed
 *   5. drive forward    FORWARD_2_SECONDS
 *   6. strafe right     STRAFE_RIGHT_SECONDS
 *   7. drive backward   BACKWARD_SECONDS
 *   8. strafe right     STRAFE_RIGHT_2_SECONDS
 */
@Autonomous(name="JoJa Auto Shoot and Back", group="Competition")
public class JoJaAutoShootAndBack extends OpMode {
    private static final double DRIVE_POWER = 0.5;
    private static final double STRAFE_LEFT_SECONDS = 2.15;
    private static final double FORWARD_SECONDS = 1;
    private static final double SPINUP_SECONDS = 2.0;
    private static final double FLYWHEEL_RPM = 3410;
    // this auto only; JoJa Tele keeps RoboConstants.FLYWHEEL_TARGET_RPM
    private static final double SHOOT_SECONDS = 3.0;    // stopper open + feeding
    private static final double FORWARD_2_SECONDS = 0.5;
    private static final double STRAFE_RIGHT_SECONDS = 1.125;
    private static final double BACKWARD_SECONDS = 2.3;
    private static final double STRAFE_RIGHT_2_SECONDS = 1.0;

    private enum Step { STRAFE_LEFT, FORWARD, SPIN_UP, SHOOT, FORWARD_2, STRAFE_RIGHT, BACKWARD, STRAFE_RIGHT_2, DONE }

    private Drivetrain drivetrain;
    private Flywheel flywheel;
    private Intake intake;
    private Indexer indexer;
    private Stopper stopper;

    private Step step = Step.STRAFE_LEFT;
    private double savedTargetRpm;
    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void init() {
        savedTargetRpm = RoboConstants.FLYWHEEL_TARGET_RPM; // restored in stop()
        drivetrain = new Drivetrain(hardwareMap,
                RoboConstants.DRIVE_LEFT_FRONT_NAME,
                RoboConstants.DRIVE_LEFT_BACK_NAME,
                RoboConstants.DRIVE_RIGHT_FRONT_NAME,
                RoboConstants.DRIVE_RIGHT_BACK_NAME, telemetry);
        flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_LEFT_MOTOR_NAME, RoboConstants.FLYWHEEL_RIGHT_MOTOR_NAME, telemetry);
        intake = new Intake(hardwareMap, RoboConstants.INTAKE_MOTOR_NAME, telemetry);
        indexer = new Indexer(hardwareMap, RoboConstants.INDEXER_MOTOR_NAME, telemetry);
        stopper = new Stopper(hardwareMap, RoboConstants.STOPPER_SERVO_NAME, telemetry); // starts CLOSED

        telemetry.addData(">", "Left, forward, shoot, forward, right, back, right. Press Play.");
    }

    @Override
    public void start() {
        startStep(Step.STRAFE_LEFT);
    }

    private void startStep(Step next) {
        step = next;
        stepTimer.reset();
        switch (next) {
            case STRAFE_LEFT:
                drivetrain.timedMove(0.0, -DRIVE_POWER, 0.0, STRAFE_LEFT_SECONDS); // strafe: + = right
                break;
            case FORWARD:
                drivetrain.timedMove(DRIVE_POWER, 0.0, 0.0, FORWARD_SECONDS);
                break;
            case SPIN_UP:
                RoboConstants.FLYWHEEL_TARGET_RPM = FLYWHEEL_RPM;
                flywheel.spinUp();
                break;
            case SHOOT:
                stopper.open();
                intake.setState(Intake.State.INTAKING);
                indexer.setState(Indexer.State.FEEDING);
                break;
            case FORWARD_2:
                flywheel.stop();
                stopper.close();
                intake.setState(Intake.State.OFF);
                indexer.setState(Indexer.State.OFF);
                drivetrain.timedMove(DRIVE_POWER, 0.0, 0.0, FORWARD_2_SECONDS);
                break;
            case STRAFE_RIGHT:
                drivetrain.timedMove(0.0, DRIVE_POWER, 0.0, STRAFE_RIGHT_SECONDS);
                break;
            case BACKWARD:
                drivetrain.timedMove(-DRIVE_POWER, 0.0, 0.0, BACKWARD_SECONDS);
                break;
            case STRAFE_RIGHT_2:
                drivetrain.timedMove(0.0, DRIVE_POWER, 0.0, STRAFE_RIGHT_2_SECONDS);
                break;
            case DONE:
                drivetrain.stop();
                break;
        }
    }

    @Override
    public void loop() {
        switch (step) {
            case STRAFE_LEFT:
                if (!drivetrain.isBusy()) startStep(Step.FORWARD);
                break;
            case FORWARD:
                if (!drivetrain.isBusy()) startStep(Step.SPIN_UP);
                break;
            case SPIN_UP:
                if (stepTimer.seconds() >= SPINUP_SECONDS) startStep(Step.SHOOT);
                break;
            case SHOOT:
                if (stepTimer.seconds() >= SHOOT_SECONDS) startStep(Step.FORWARD_2);
                break;
            case FORWARD_2:
                if (!drivetrain.isBusy()) startStep(Step.STRAFE_RIGHT);
                break;
            case STRAFE_RIGHT:
                if (!drivetrain.isBusy()) startStep(Step.BACKWARD);
                break;
            case BACKWARD:
                if (!drivetrain.isBusy()) startStep(Step.STRAFE_RIGHT_2);
                break;
            case STRAFE_RIGHT_2:
                if (!drivetrain.isBusy()) startStep(Step.DONE);
                break;
            case DONE:
                break;
        }

        drivetrain.periodic();
        flywheel.periodic();
        intake.periodic();
        indexer.periodic();
        stopper.periodic();

        telemetry.addData("Step", step);
        telemetry.addData("Step time", "%.1f s", stepTimer.seconds());
    }

    @Override
    public void stop() {
        RoboConstants.FLYWHEEL_TARGET_RPM = savedTargetRpm; // put TeleOp's target back
        drivetrain.stop();
        flywheel.stop();
        intake.setState(Intake.State.OFF);
        indexer.setState(Indexer.State.OFF);
    }
}
