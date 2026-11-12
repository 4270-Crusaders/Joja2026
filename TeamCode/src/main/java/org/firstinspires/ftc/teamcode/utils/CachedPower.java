package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Sends motor power only when it really changed (more than MOTOR_WRITE_DEADBAND). Every setPower is its own
 * hub command (~1-3 ms, slower on the Expansion Hub), so skipping tiny changes keeps the loop fast.
 * 0 and full power are always sent exactly.
 */
public final class CachedPower {
    private final DcMotorSimple motor;
    private double last = Double.NaN;

    public CachedPower(DcMotorSimple motor) { this.motor = motor; }

    public void set(double power) {
        boolean exactEnd = (power == 0.0 || Math.abs(power) == 1.0) && power != last;
        if (Double.isNaN(last) || exactEnd || Math.abs(power - last) > RoboConstants.MOTOR_WRITE_DEADBAND) {
            motor.setPower(power);
            last = power;
        }
    }

    public double get() { return Double.isNaN(last) ? 0.0 : last; }

    /** Forget the last value, so the next set() always writes (use after something else drove this motor). */
    public void reset() { last = Double.NaN; }
}
