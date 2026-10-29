package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Dumper-style indexer: holds 4 balls, takes in everything, shoots everything at once.
 * Plain open-loop power, no sensors. The OpMode sets the state every loop
 * (see Coordination in .pm/backlog.md); this class only maps state -> power.
 */
public class Indexer extends SubsystemBase {
    public enum State {
        OFF,
        INTAKING,
        FEEDING,
        REVERSING
    }

    Telemetry telemetry;
    private final DcMotorEx motor;
    private final VoltageSensor voltageSensor;
    private State state = State.OFF;

    public Indexer(final HardwareMap hMap, final String motorName, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = hMap.get(DcMotorEx.class, motorName);
        motor.setDirection(RoboConstants.INDEXER_DIRECTION);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // keep balls from drifting into the stopper
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
        java.util.Iterator<VoltageSensor> it = hMap.voltageSensor.iterator();
        voltageSensor = it.hasNext() ? it.next() : null;
        RobotLog.i("[Indexer] Initialized.");
    }

    public void setState(State state) { this.state = state; }
    public State getState() { return state; }

//    /** Power that puts about INDEXER_FEED_VOLTS on the motor at the current battery voltage. */
//    private double feedPower() {
//        double battery = (voltageSensor != null) ? voltageSensor.getVoltage() : 12.0;
//        if (battery < 1.0) battery = 12.0;
//        return Math.max(0.0, Math.min(1.0, RoboConstants.INDEXER_FEED_VOLTS / battery));
//    }

    private double powerFor(State s) {
        switch (s) {
            case INTAKING:  return RoboConstants.INDEXER_INTAKE_POWER;
            case FEEDING:   return RoboConstants.INDEXER_FEED_POWER;
            case REVERSING: return RoboConstants.INDEXER_REVERSE_POWER;
            case OFF:
            default:        return 0.0;
        }
    }

    @Override
    public void periodic() {
        // Read constants every loop so Panels edits take effect live.
        double power = powerFor(state);
        motor.setPower(power);

        telemetry.addLine("====== Indexer ======");
        telemetry.addData("Indexer/State", state);
        telemetry.addData("Indexer/Power", "%.2f", power);
    }
}
