package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

import java.util.ArrayList;
import java.util.List;

/**
 * Timed auto (no Pedro): shoot the preloads, then collect POLLEN from a FLOWER.
 * Steps 1-4 are the tuned steps from JoJa Auto Shoot and Back.
 *
 * To add more commands, add lines to buildSteps() using the helpers below
 * (strafeLeft, strafeRight, forward, backward, turnClockwise, turnCounterClockwise, waitFor,
 *  spinUpFlywheel, shoot, matchLoaderDown, matchLoaderMiddle, matchLoaderUp, intakeOn, intakeOff).
 */
@Autonomous(name="JoJa Auto Shoot and Collect", group="Competition")
public class JoJaAutoShootAndCollect extends OpMode {
    private static final double DRIVE_POWER = 0.5;
    private static final double TURN_POWER = 0.4;
    private static final double TURN_90_SECONDS = 0.5;   // GUESS: time for a 90 deg turn at TURN_POWER. TUNE on the robot
    private static final double FLYWHEEL_RPM = 3400.0;   // this auto only; JoJa Tele keeps RoboConstants.FLYWHEEL_TARGET_RPM

    private void buildSteps() {
        // 1-4: same as JoJa Auto Shoot and Back (tuned)
        strafeLeft(2.15);
        forward(1.0);
        spinUpFlywheel(2.0);       // flywheel on, wait 2 s
        shoot(3.0);                // stopper open + feed 3 s, then flywheel off, stopper closed

        // 5: new
        forward(0.6);

        // Collect POLLEN from the FLOWER
        backward(1.2);
        turnClockwise(TURN_90_SECONDS);
        matchLoaderDown(0.5);      // deploy, give the servo 0.5 s to get there
        intakeOn();
        forward(2.0);              // drive into the FLOWER while intaking
        intakeOff();

        // Add more commands here
    }

    // ===================================================================================
    // Step machinery: you shouldn't need to change anything below this line
    // ===================================================================================

    /** One step: start() runs once, isDone() is checked every loop until it returns true. */
    private interface Step {
        void start();
        boolean isDone();
    }

    private Drivetrain drivetrain;
    private Flywheel flywheel;
    private Intake intake;
    private Indexer indexer;
    private Stopper stopper;
    private MatchLoader matchLoader;

    private final List<Step> steps = new ArrayList<>();
    private int stepIndex = 0;
    private boolean stepStarted = false;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private double savedTargetRpm;

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
        stopper = new Stopper(hardwareMap, RoboConstants.STOPPER_SERVO_NAME, telemetry);             // starts CLOSED
        matchLoader = new MatchLoader(hardwareMap, RoboConstants.MATCH_LOADER_SERVO_NAME, telemetry); // starts UP

        buildSteps();
        telemetry.addData(">", "%d steps loaded. Press Play.", steps.size());
    }

    @Override
    public void loop() {
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

        drivetrain.periodic();
        flywheel.periodic();
        intake.periodic();
        indexer.periodic();
        stopper.periodic();
        matchLoader.periodic();

        telemetry.addData("Step", stepIndex < steps.size() ? (stepIndex + 1) + " / " + steps.size() : "DONE");
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

    // ===== Step helpers =====

    /** Timed, voltage-compensated drive. fwd + = forward, strafe + = right, turn + = clockwise. */
    private void drive(final double fwd, final double strafe, final double turn, final double seconds) {
        steps.add(new Step() {
            public void start() { drivetrain.timedMove(fwd, strafe, turn, seconds); }
            public boolean isDone() { return !drivetrain.isBusy(); }
        });
    }

    private void forward(double seconds)              { drive(DRIVE_POWER, 0, 0, seconds); }
    private void backward(double seconds)             { drive(-DRIVE_POWER, 0, 0, seconds); }
    private void strafeRight(double seconds)          { drive(0, DRIVE_POWER, 0, seconds); }
    private void strafeLeft(double seconds)           { drive(0, -DRIVE_POWER, 0, seconds); }
    private void turnClockwise(double seconds)        { drive(0, 0, TURN_POWER, seconds); }
    private void turnCounterClockwise(double seconds) { drive(0, 0, -TURN_POWER, seconds); }

    private void waitFor(final double seconds) {
        steps.add(new Step() {
            public void start() { }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    /** Flywheel on at FLYWHEEL_RPM, then wait {@code seconds} for it to spin up. */
    private void spinUpFlywheel(final double seconds) {
        steps.add(new Step() {
            public void start() {
                RoboConstants.FLYWHEEL_TARGET_RPM = FLYWHEEL_RPM;
                flywheel.spinUp();
            }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    /** Stopper open + intake + indexer feed for {@code seconds}; then flywheel off, stopper closed, feed off. */
    private void shoot(final double seconds) {
        steps.add(new Step() {
            public void start() {
                stopper.open();
                intake.setState(Intake.State.INTAKING);
                indexer.setState(Indexer.State.FEEDING);
            }
            public boolean isDone() {
                if (stepTimer.seconds() < seconds) return false;
                flywheel.stop();
                stopper.close();
                intake.setState(Intake.State.OFF);
                indexer.setState(Indexer.State.OFF);
                return true;
            }
        });
    }

    /** Match loader DOWN (deployed), then wait {@code seconds} for the servo to get there. */
    private void matchLoaderDown(final double seconds) {
        steps.add(new Step() {
            public void start() { matchLoader.down(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    /** Match loader MIDDLE (pollen only, no nectar), then wait {@code seconds} for the servo to get there. */
    private void matchLoaderMiddle(final double seconds) {
        steps.add(new Step() {
            public void start() { matchLoader.middle(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    /** Match loader UP (closed), then wait {@code seconds} for the servo to get there. */
    private void matchLoaderUp(final double seconds) {
        steps.add(new Step() {
            public void start() { matchLoader.up(); }
            public boolean isDone() { return stepTimer.seconds() >= seconds; }
        });
    }

    /** Intake + indexer on (collect speed) with the stopper closed so balls stay in. Instant. */
    private void intakeOn() {
        steps.add(new Step() {
            public void start() {
                stopper.close();
                intake.setState(Intake.State.INTAKING);
                indexer.setState(Indexer.State.INTAKING);
            }
            public boolean isDone() { return true; }
        });
    }

    /** Intake + indexer off. Instant. */
    private void intakeOff() {
        steps.add(new Step() {
            public void start() {
                intake.setState(Intake.State.OFF);
                indexer.setState(Indexer.State.OFF);
            }
            public boolean isDone() { return true; }
        });
    }
}
