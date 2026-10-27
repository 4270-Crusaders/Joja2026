package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

public class Flywheel extends SubsystemBase {
    public enum State {
        OFF,
        SPINNING_UP,
        AT_SPEED
    }

    Telemetry telemetry;
    private final DcMotorEx a, b;
    private State state = State.OFF;
    private double prevErr = 0.0;

    public Flywheel(final HardwareMap hMap, final String flywheelAName, final String flywheelBName, Telemetry telemetry) {
        this.telemetry = telemetry;
        a = hMap.get(DcMotorEx.class, flywheelAName);
        b = hMap.get(DcMotorEx.class, flywheelBName);
        a.setDirection(RoboConstants.FLYWHEEL_A_DIRECTION);
        b.setDirection(RoboConstants.FLYWHEEL_B_DIRECTION);
        for (DcMotorEx m : new DcMotorEx[]{a, b}) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // we run our own velocity loop
            m.setPower(0.0);
        }
        RobotLog.i("[Flywheel] Initialized.");
    }

    /** OFF -> SPINNING_UP. Safe to call every loop; no effect if already spinning. */
    public void spinUp() {
        if (state == State.OFF) {
            // Seed prevErr so the first derivative term doesn't spike.
            prevErr = RoboConstants.FLYWHEEL_TARGET_RPM - getRpm();
            state = State.SPINNING_UP;
        }
    }

    /** any -> OFF. Power 0. Safe to call every loop. */
    public void stop() {
        state = State.OFF;
        prevErr = 0.0;
    }

    /** Open-loop power to both motors, bypassing the PID. For SysId only; don't call periodic() alongside it. */
    public void setRawPower(double pwr) {
        a.setPower(pwr);
        b.setPower(pwr);
    }

    public State getState() { return state; }

    public boolean isAtSpeed() { return state == State.AT_SPEED; }

    public double getTargetRpm() { return RoboConstants.FLYWHEEL_TARGET_RPM; }

    /** Wheel RPM = motor RPM / gear ratio (gear ratio = motor revs per wheel rev). */
    public double getRpm() {
        double tps = a.getVelocity(); // only flywheelA has an encoder cable; ticks/s
        double motorRpm = (tps / RoboConstants.FLYWHEEL_TPR) * 60.0;
        return motorRpm / RoboConstants.FLYWHEEL_GEAR_RATIO;
    }

    double dt = 1.0 / RoboConstants.LOOP_HZ; // FTC OpMode loop ~ 50Hz
    @Override
    public void periodic() {
        double target = RoboConstants.FLYWHEEL_TARGET_RPM;
        double rpm = getRpm();
        double err = target - rpm;
        double pwr;

        if (state == State.OFF) {
            prevErr = 0.0;
            pwr = 0.0;
        } else {
            double deriv = (err - prevErr) / dt;
            prevErr = err;

            pwr = RoboConstants.FLYWHEEL_kP * err
                    + RoboConstants.FLYWHEEL_kD * deriv
                    + RoboConstants.FLYWHEEL_kS + RoboConstants.FLYWHEEL_kV * target;

            pwr = Math.max(0.0, Math.min(1.0, pwr)); // no reverse

            state = (Math.abs(err) <= RoboConstants.FLYWHEEL_RPM_TOLERANCE)
                    ? State.AT_SPEED
                    : State.SPINNING_UP;
        }

        a.setPower(pwr);
        b.setPower(pwr);

        telemetry.addLine("====== Flywheel ======");
        telemetry.addData("Flywheel/State", state);
        telemetry.addData("Flywheel/TargetRPM", "%.0f", target);
        telemetry.addData("Flywheel/RPM", "%.0f", rpm);
        telemetry.addData("Flywheel/Error", "%.0f", err);
        telemetry.addData("Flywheel/Power", "%.2f", pwr);
    }
}
