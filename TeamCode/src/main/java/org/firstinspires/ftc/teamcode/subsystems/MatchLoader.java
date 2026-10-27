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
 *
 * States: DOWN (initial) and UP. up()/down()/toggle() only store the state;
 * the servo is written in periodic().
 *
 * While UP, the OpMode runs Intake + Indexer INTAKING (coordination lives in the OpMode).
 */
public class MatchLoader extends SubsystemBase {
    public enum State {
        DOWN,
        UP
    }

    Telemetry telemetry;
    private final Servo matchLoader;
    private State state = State.DOWN;

    public MatchLoader(final HardwareMap hMap, final String servoName, Telemetry telemetry) {
        this.telemetry = telemetry;
        matchLoader = hMap.get(Servo.class, servoName);
        // Hold the plate DOWN from init (same as Stopper).
        matchLoader.setPosition(RoboConstants.MATCH_LOADER_DOWN_POS);
        RobotLog.i("[MatchLoader] Initialized.");
    }

    public void up() {
        state = State.UP;
    }

    public void down() {
        state = State.DOWN;
    }

    public void toggle() {
        state = (state == State.UP) ? State.DOWN : State.UP;
    }

    public State getState() {
        return state;
    }

    public boolean isUp() {
        return state == State.UP;
    }

    private double getTargetPosition() {
        return (state == State.UP) ? RoboConstants.MATCH_LOADER_UP_POS : RoboConstants.MATCH_LOADER_DOWN_POS;
    }

    @Override
    public void periodic() {
        double target = getTargetPosition();
        matchLoader.setPosition(target);

        telemetry.addLine("====== Match Loader ======");
        telemetry.addData("MatchLoader/State", state);
        telemetry.addData("MatchLoader/TargetPos", target);
        telemetry.addData("MatchLoader/CommandedPos", matchLoader.getPosition());
    }
}
