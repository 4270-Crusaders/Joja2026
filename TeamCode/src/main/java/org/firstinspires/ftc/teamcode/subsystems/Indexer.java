package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.utils.Battery;
import org.firstinspires.ftc.teamcode.utils.CachedPower;
import org.firstinspires.ftc.teamcode.utils.LoopTimer;

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
        FEEDING_SLOW, // shooting, but the flywheel is recovering from the last ball: keep moving slowly, don't stop
        REVERSING
    }

    Telemetry telemetry;
    private final DcMotorEx motor;
    private final CachedPower out;
    private State state = State.OFF;
    private double appliedPower = 0.0;              // ramped power actually sent to the motor
    private double feedScale = 1.0;                 // FEEDING: 1 = full feed power, 0 = INDEXER_FEED_SLOW_POWER
    private final ElapsedTime rampTimer = new ElapsedTime();

    public Indexer(final HardwareMap hMap, final String motorName, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = hMap.get(DcMotorEx.class, motorName);
        motor.setDirection(RoboConstants.INDEXER_DIRECTION);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // keep balls from drifting into the stopper
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
        out = new CachedPower(motor);
        Battery.init(hMap); // shared battery reading (voltage compensation)
        RobotLog.i("[Indexer] Initialized.");
    }

    public void setState(State state) { this.state = state; }

    /** Proportional feed (Flywheel.getFeedScale()): FEEDING power goes from FEED_SLOW (0) to FEED (1). */
    public void setFeedScale(double scale) { feedScale = Math.max(0.0, Math.min(1.0, scale)); }
    public State getState() { return state; }

    private double powerFor(State s) {
        switch (s) {
            case INTAKING:     return RoboConstants.INDEXER_INTAKE_POWER;
            case FEEDING:      return RoboConstants.INDEXER_FEED_SLOW_POWER
                    + (RoboConstants.INDEXER_FEED_POWER - RoboConstants.INDEXER_FEED_SLOW_POWER) * feedScale;
            case FEEDING_SLOW: return RoboConstants.INDEXER_FEED_SLOW_POWER;
            case REVERSING:    return RoboConstants.INDEXER_REVERSE_POWER;
            case OFF:
            default:           return 0.0;
        }
    }

    @Override
    public void periodic() {
        // Read constants every loop so Panels edits take effect live.
        // Voltage compensation: same speed at any battery level (2026-11-09). 1.0 stays 1.0.
        double target = Battery.comp(powerFor(state));
        double dt = Math.min(rampTimer.seconds(), 0.1); // cap: a long gap between loops shouldn't skip the ramp
        rampTimer.reset();

        // Ramp: speeding up is limited to INDEXER_RAMP_PER_SEC (no current spike); slowing down / stopping is instant.
        // Changing direction (e.g. feed -> reverse) drops to 0 first, then ramps up the other way.
        boolean sameDirection = target * appliedPower >= 0;
        if (!sameDirection) appliedPower = 0.0;
        if (Math.abs(target) <= Math.abs(appliedPower) || RoboConstants.INDEXER_RAMP_PER_SEC <= 0) {
            appliedPower = target;
        } else {
            double step = RoboConstants.INDEXER_RAMP_PER_SEC * dt;
            appliedPower += Math.signum(target) * Math.min(step, Math.abs(target - appliedPower));
        }
        out.set(appliedPower);

        if (!LoopTimer.telemetryNow()) return; // telemetry only every TELEMETRY_PERIOD_MS
        telemetry.addLine("====== Indexer ======");
        telemetry.addData("Indexer/State", state);
        telemetry.addData("Indexer/Power", "%.2f", appliedPower);
        telemetry.addData("Indexer/Target power", "%.2f", target);
    }
}
