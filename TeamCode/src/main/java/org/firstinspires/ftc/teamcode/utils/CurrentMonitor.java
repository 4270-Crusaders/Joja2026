package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RoboConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Amps per motor, to see where the battery's current goes (RoboConstants.CURRENT_MONITOR).
 *
 * Reading a motor's current is its own hub command (~1-2 ms; not part of the bulk read), so this reads ONE motor
 * per loop, round-robin. With 8 motors each one updates about every 8 loops (~60 ms at a 7 ms loop).
 * Shows "now" and the PEAK in the last PEAK_WINDOW_S (the peak during a volley is the useful number).
 */
public final class CurrentMonitor {
    private static final double PEAK_WINDOW_S = 2.0;

    private static final class Entry {
        final String name;
        final DcMotorEx motor;
        double amps = 0.0, peak = 0.0, shownPeak = 0.0;
        Entry(String name, DcMotorEx motor) { this.name = name; this.motor = motor; }
    }

    private final List<Entry> entries = new ArrayList<>();
    private final ElapsedTime peakTimer = new ElapsedTime();
    private int next = 0;

    /** Add a motor by its config name. Unknown names are skipped. */
    public CurrentMonitor add(HardwareMap hMap, String label, String configName) {
        try {
            entries.add(new Entry(label, hMap.get(DcMotorEx.class, configName)));
        } catch (Exception ignored) { }
        return this;
    }

    /** Call once per loop. Does nothing when CURRENT_MONITOR is off. */
    public void update() {
        if (!RoboConstants.CURRENT_MONITOR || entries.isEmpty()) return;
        Entry e = entries.get(next);
        next = (next + 1) % entries.size();
        e.amps = e.motor.getCurrent(CurrentUnit.AMPS);
        e.peak = Math.max(e.peak, e.amps);
        if (peakTimer.seconds() > PEAK_WINDOW_S) {
            for (Entry x : entries) { x.shownPeak = x.peak; x.peak = x.amps; }
            peakTimer.reset();
        }
    }

    /** One number per line, so Panels can graph them. */
    public void addTelemetry(Telemetry t) {
        if (!RoboConstants.CURRENT_MONITOR) return;
        double total = 0.0;
        for (Entry e : entries) {
            t.addData("Amps/" + e.name, "%.1f", e.amps);
            t.addData("Amps/" + e.name + " peak", "%.1f", Math.max(e.shownPeak, e.peak));
            total += e.amps;
        }
        t.addData("Amps/Total", "%.1f", total);
    }
}
