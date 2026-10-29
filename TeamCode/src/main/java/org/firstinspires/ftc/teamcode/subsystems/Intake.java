package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Intake: single goBILDA 5203 1150 RPM motor, plain open-loop power.
 * The OpMode sets the state every loop (see Coordination in .pm/backlog.md);
 * the hardware write happens in periodic().
 */
public class Intake extends SubsystemBase {
    public enum State {
        OFF,
        INTAKING,
        REVERSING
    }

    Telemetry telemetry;
    private final DcMotorEx motor;
    private State state = State.OFF;

    public Intake(final HardwareMap hMap, final String motorName, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = hMap.get(DcMotorEx.class, motorName);
        motor.setDirection(RoboConstants.INTAKE_DIRECTION);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
        RobotLog.i("[Intake] Initialized.");
    }

    public void setState(State state) { this.state = state; }
    public State getState() { return state; }

    @Override
    public void periodic() {
        // Power is looked up every loop so Panels edits to RoboConstants take effect live.
        double power;
        switch (state) {
            case INTAKING:
                power = RoboConstants.INTAKE_POWER;
                break;
            case REVERSING:
                power = RoboConstants.INTAKE_REVERSE_POWER;
                break;
            case OFF:
            default:
                power = 0.0;
                break;
        }

        motor.setPower(power);

        telemetry.addLine("====== Intake ======");
        telemetry.addData("Intake/State", state);
        telemetry.addData("Intake/Power", "%.2f", power);
    }
}
