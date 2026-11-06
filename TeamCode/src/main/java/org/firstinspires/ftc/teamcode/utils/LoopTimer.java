package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Fast control loop (target 5-10 ms per loop).
 *
 * Call tick() once at the START of every loop. It:
 *   1. waits until LOOP_MIN_PERIOD_MS has passed since the last loop (even, steady loop time; the flywheel's
 *      accel / integral math works best when every loop is about the same length)
 *   2. decides if THIS loop sends telemetry. Building telemetry text and sending it to the Driver Hub / Panels
 *      takes several ms, so it only happens every TELEMETRY_PERIOD_MS. Control runs every loop.
 *
 * Subsystems check telemetryNow() and skip their telemetry on the other loops. OpModes that don't use a
 * LoopTimer (tests, old autos) still get telemetry every loop.
 */
public final class LoopTimer {
    private static boolean telemetryThisLoop = true;
    private static final ElapsedTime sinceAnyTick = new ElapsedTime();

    /** true if telemetry should be built this loop. Always true when no LoopTimer is running. */
    public static boolean telemetryNow() {
        return telemetryThisLoop || sinceAnyTick.milliseconds() > 200;
    }

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime teleTimer = new ElapsedTime();
    private final ElapsedTime maxWindow = new ElapsedTime();
    private double loopMs = 0.0, avgMs = 0.0, maxMs = 0.0, maxShownMs = 0.0;

    public LoopTimer() { telemetryThisLoop = true; }

    /** Call at the very start of loop(). */
    public void tick() {
        double minMs = RoboConstants.LOOP_MIN_PERIOD_MS;
        double left = minMs - loopTimer.milliseconds();
        if (left > 0.5) {
            try { Thread.sleep((long) left); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        }
        while (loopTimer.milliseconds() < minMs) { /* last fraction of a ms */ }

        loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        avgMs = (avgMs == 0.0) ? loopMs : 0.95 * avgMs + 0.05 * loopMs;
        maxMs = Math.max(maxMs, loopMs);
        if (maxWindow.seconds() > 1.0) { maxShownMs = maxMs; maxMs = 0.0; maxWindow.reset(); }

        sinceAnyTick.reset();
        telemetryThisLoop = teleTimer.milliseconds() >= RoboConstants.TELEMETRY_PERIOD_MS;
        if (telemetryThisLoop) teleTimer.reset();
    }

    /** Call in the OpMode's stop(), so other OpModes get telemetry every loop again. */
    public static void release() { telemetryThisLoop = true; }

    public double getLoopMs() { return loopMs; }
    public double getAverageMs() { return avgMs; }
    /** Slowest loop in the last second. */
    public double getMaxMs() { return maxShownMs; }
}
