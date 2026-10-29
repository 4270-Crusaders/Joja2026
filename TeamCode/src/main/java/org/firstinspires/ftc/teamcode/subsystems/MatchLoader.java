package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Match Loader: one positional servo that lifts a plate so the robot can collect
 * balls out of a FLOWER.
 * States:
 *   UP     closed / resting (initial)
 *   DOWN   all the way down (deployed: collecting from a FLOWER)
 *   MIDDLE in between: gap only lets POLLEN through, not NECTAR
 * up()/down()/middle()/toggle() only store the state; the servo is written in periodic().
 * It does not start the intake.
 */
public class MatchLoader extends SubsystemBase {
    public enum State {
        DOWN,
        MIDDLE,
        UP
    }

    Telemetry telemetry;
    private final Servo matchLoader;
    private State state = State.UP;

    public MatchLoader(final HardwareMap hMap, final String servoName, Telemetry telemetry) {
        this.telemetry = telemetry;
        matchLoader = hMap.get(Servo.class, servoName);
        // Hold the plate DOWN from init (same as Stopper).
        matchLoader.setPosition(getTargetPosition()); // start closed (UP + offset)
        RobotLog.i("[MatchLoader] Initialized.");
    }

    public void up() {
        state = State.UP;
    }

    public void down() {
        state = State.DOWN;
    }

    /** Pollen-only position (between UP and DOWN). */
    public void middle() {
        state = State.MIDDLE;
    }

    /** UP -> DOWN; DOWN or MIDDLE -> UP. */
    public void toggle() {
        state = (state == State.UP) ? State.DOWN : State.UP;
    }

    /** UP or DOWN -> MIDDLE; MIDDLE -> UP. */
    public void toggleMiddle() {
        state = (state == State.MIDDLE) ? State.UP : State.MIDDLE;
    }

    public boolean isMiddle() {
        return state == State.MIDDLE;
    }

    public State getState() {
        return state;
    }

    public boolean isUp() {
        return state == State.UP;
    }

    /** DOWN = deployed, collecting balls from a FLOWER. */
    public boolean isDeployed() {
        return state == State.DOWN;
    }

    private double getTargetPosition() {
        double base;
        switch (state) {
            case DOWN:   base = RoboConstants.MATCH_LOADER_DOWN_POS;   break;
            case MIDDLE: base = RoboConstants.MATCH_LOADER_MIDDLE_POS; break;
            default:     base = RoboConstants.MATCH_LOADER_UP_POS;     break;
        }
        // offset shifts every position by the same amount (horn mounted at the wrong angle)
        return Math.max(0.0, Math.min(1.0, base + RoboConstants.MATCH_LOADER_OFFSET));
    }

    /** Shift all three positions by {@code delta} (D-pad trim in TeleOp). */
    public void adjustOffset(double delta) {
        RoboConstants.MATCH_LOADER_OFFSET += delta;
    }

    @Override
    public void periodic() {
        double target = getTargetPosition();
        matchLoader.setPosition(target);

        telemetry.addLine("====== Match Loader ======");
        telemetry.addData("MatchLoader/State", state);
        telemetry.addData("MatchLoader/TargetPos", target);
        telemetry.addData("MatchLoader/Offset", "%.2f", RoboConstants.MATCH_LOADER_OFFSET);
        telemetry.addData("MatchLoader/CommandedPos", matchLoader.getPosition());
    }
}
