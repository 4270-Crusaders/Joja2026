package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Ball stopper: a single positional servo that blocks balls from reaching the flywheel.
 * The OpMode decides when to open or close it (gated on the Flywheel being AT_SPEED);
 * this class only stores the requested state and writes the servo in periodic().
 */
public class Stopper extends SubsystemBase {

    public enum State {
        CLOSED,
        OPEN
    }

    Telemetry telemetry;
    private final Servo stopper;
    private State state = State.CLOSED;

    public Stopper(final HardwareMap hMap, final String servoName, Telemetry telemetry) {
        this.telemetry = telemetry;
        stopper = hMap.get(Servo.class, servoName);
        // Hold the preloads from init: command CLOSED right away (same as the reference Indexer).
        stopper.setPosition(RoboConstants.STOPPER_CLOSED_POS);
        RobotLog.i("[Stopper] Initialized.");
    }

    public void open() {
        state = State.OPEN;
    }

    public void close() {
        state = State.CLOSED;
    }

    public State getState() {
        return state;
    }

    public boolean isOpen() {
        return state == State.OPEN;
    }

    private double getTargetPosition() {
        return state == State.OPEN ? RoboConstants.STOPPER_OPEN_POS : RoboConstants.STOPPER_CLOSED_POS;
    }

    @Override
    public void periodic() {
        double target = getTargetPosition();
        stopper.setPosition(target);

        telemetry.addLine("====== Stopper ======");
        telemetry.addData("Stopper/State", state);
        telemetry.addData("Stopper/TargetPos", target);
        telemetry.addData("Stopper/ServoPos", stopper.getPosition());
    }
}
