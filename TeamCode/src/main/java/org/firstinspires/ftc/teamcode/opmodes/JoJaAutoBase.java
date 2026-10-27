package org.firstinspires.ftc.teamcode.opmodes;

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
 * Shared time + voltage based auto sequencer. Subclasses list their steps in buildSteps().
 * Alliance is picked during init (gamepad1 X = Blue, B = Red); Red mirrors strafe and turn.
 */
public abstract class JoJaAutoBase extends OpMode {
    public enum Alliance { BLUE, RED }

    // If the flywheel never reports AT_SPEED (e.g. target RPM unreachable), shoot anyway after this
    public static double SHOOT_SPINUP_TIMEOUT_S = 2.5;

    /** One step of the sequence. start() runs once, update() runs every loop until it returns true. */
    protected interface Step {
        void start();
        boolean update();
    }

    // ===== Subsystems =====
    protected Drivetrain drivetrain;
    protected Intake intake;
    protected Indexer indexer;
    protected Stopper stopper;
    protected Flywheel flywheel;
    protected MatchLoader matchLoader;

    protected Alliance alliance = Alliance.BLUE;

    private final List<Step> steps = new ArrayList<>();
    private int stepIndex = 0;
    private boolean stepStarted = false;
    private final ElapsedTime autoTimer = new ElapsedTime();

    /** Add this routine's steps with the drive/spinUp/shoot/... helpers below. */
    protected abstract void buildSteps();

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
        flywheel = new Flywheel(hardwareMap, RoboConstants.FLYWHEEL_A_MOTOR_NAME, RoboConstants.FLYWHEEL_B_MOTOR_NAME, telemetry);
        matchLoader = new MatchLoader(hardwareMap, RoboConstants.MATCH_LOADER_SERVO_NAME, telemetry);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        telemetry.addData("Alliance", "%s  (X = Blue, B = Red)", alliance);
        telemetry.addData(">", "Robot Ready.  Press Play.");
    }

    @Override
    public void start() {
        steps.clear();
        buildSteps();  // built at start so the steps see the chosen alliance
        stepIndex = 0;
        stepStarted = false;
        autoTimer.reset();
    }

    @Override
    public void loop() {
        if (stepIndex < steps.size()) {
            Step step = steps.get(stepIndex);
            if (!stepStarted) {
                step.start();
                stepStarted = true;
            }
            if (step.update()) {
                stepIndex++;
                stepStarted = false;
            }
        }

        // subsystem
        drivetrain.periodic();
        flywheel.periodic();
        intake.periodic();
        indexer.periodic();
        stopper.periodic();
        matchLoader.periodic();

        telemetry.addData("Auto/Alliance", alliance);
        telemetry.addData("Auto/Step", "%d / %d", Math.min(stepIndex + 1, steps.size()), steps.size());
        telemetry.addData("Auto/Time", "%.1f s", autoTimer.seconds());
    }

    @Override
    public void stop() {
        drivetrain.stop();
        flywheel.stop();
        intake.setState(Intake.State.OFF);
        indexer.setState(Indexer.State.OFF);
    }

    // ===== Step helpers =====

    private double mirror() { return alliance == Alliance.RED ? -1.0 : 1.0; }

    /** Timed, voltage-compensated drive. fwd +, strafe right +, turn clockwise + (Blue frame). */
    protected void drive(final double fwd, final double strafe, final double turn, final double seconds) {
        steps.add(new Step() {
            public void start() { drivetrain.timedMove(fwd, strafe * mirror(), turn * mirror(), seconds); }
            public boolean update() { return !drivetrain.isBusy(); }
        });
    }

    protected void spinUp() {
        steps.add(new Step() {
            public void start() { flywheel.spinUp(); }
            public boolean update() { return true; }
        });
    }

    protected void stopFlywheel() {
        steps.add(new Step() {
            public void start() { flywheel.stop(); }
            public boolean update() { return true; }
        });
    }

    /** Waits for AT_SPEED (or the spin-up timeout), then opens the stopper and feeds for {@code seconds}. */
    protected void shoot(final double seconds) {
        steps.add(new Step() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean latched;

            public void start() {
                flywheel.spinUp();
                latched = false;
                timer.reset();
            }

            public boolean update() {
                if (!latched && (flywheel.isAtSpeed() || timer.seconds() >= SHOOT_SPINUP_TIMEOUT_S)) {
                    latched = true;
                    timer.reset();
                }
                if (!latched) {
                    stopper.close();
                    indexer.setState(Indexer.State.OFF);
                    intake.setState(Intake.State.OFF);
                    return false;
                }
                if (timer.seconds() < seconds) {
                    stopper.open();
                    indexer.setState(Indexer.State.FEEDING);
                    intake.setState(Intake.State.INTAKING);
                    return false;
                }
                stopper.close();
                indexer.setState(Indexer.State.OFF);
                intake.setState(Intake.State.OFF);
                return true;
            }
        });
    }

    protected void waitFor(final double seconds) {
        steps.add(new Step() {
            private final ElapsedTime timer = new ElapsedTime();
            public void start() { timer.reset(); }
            public boolean update() { return timer.seconds() >= seconds; }
        });
    }

    protected void matchLoaderUp() {
        steps.add(new Step() {
            public void start() { matchLoader.up(); }
            public boolean update() { return true; }
        });
    }

    protected void matchLoaderDown() {
        steps.add(new Step() {
            public void start() { matchLoader.down(); }
            public boolean update() { return true; }
        });
    }
}
