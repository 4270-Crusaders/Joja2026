package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoboConstants;

import java.util.Iterator;

/**
 * One shared battery reading for every subsystem (2026-11-09).
 *
 * Voltage compensation: a motor's speed depends on the VOLTS it gets, and power 0..1 is a fraction of the battery.
 * So the same power gives less speed as the battery drains (13 V fresh -> ~11.5 V late in a match).
 * comp(power) = power * BATTERY_NOMINAL_VOLTAGE / battery keeps the volts (and speed) the same at any charge.
 * It can't go above 1.0: once a battery is too low for the volts a motor needs, nothing in code can add more.
 *
 * Reading the battery is its own hub command (~1-2 ms), so it's read at most every PERIOD_S and smoothed,
 * and all subsystems share that one reading instead of each reading it themselves.
 */
public final class Battery {
    private static final double PERIOD_S = 0.1;
    private static VoltageSensor sensor;
    private static double volts = 12.0;
    private static final ElapsedTime timer = new ElapsedTime();

    private Battery() { }

    /** Call in each subsystem's constructor (cheap; safe to call more than once). */
    public static void init(HardwareMap hMap) {
        Iterator<VoltageSensor> it = hMap.voltageSensor.iterator();
        sensor = it.hasNext() ? it.next() : null;
        if (sensor != null) {
            double v = sensor.getVoltage();
            if (v > 6.0) volts = v;
        }
        timer.reset();
    }

    /** Smoothed battery voltage (re-read at most every PERIOD_S). */
    public static double volts() {
        if (sensor != null && timer.seconds() >= PERIOD_S) {
            timer.reset();
            double v = sensor.getVoltage();
            if (v > 6.0) volts = 0.7 * volts + 0.3 * v; // smooth out spikes
        }
        return volts;
    }

    /** Multiply a power by this to get the same volts at any battery level (1.0 at nominal). */
    public static double scale() {
        if (!RoboConstants.BATTERY_COMP_ENABLED) return 1.0;
        return RoboConstants.BATTERY_NOMINAL_VOLTAGE / volts();
    }

    /** power * scale(), clamped to -1..1. */
    public static double comp(double power) {
        return Math.max(-1.0, Math.min(1.0, power * scale()));
    }

    /** true when the battery is too low to hold full performance (shown as a warning). */
    public static boolean isLow() { return volts < RoboConstants.BATTERY_LOW_WARNING_V; }
}
