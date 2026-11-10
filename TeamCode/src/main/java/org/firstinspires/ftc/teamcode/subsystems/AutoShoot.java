package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Automatic shooting (2026-11-09). Opens the stopper by itself once the flywheel is at target RPM,
 * and closes it again when the balls are gone. Once per right-trigger press.
 *
 *   IDLE      flywheel off. Pressing RT starts WAITING.
 *   WAITING   both sides within AUTO_SHOOT_READY_RPM of target for AUTO_SHOOT_READY_S -> open stopper, SHOOTING
 *   SHOOTING  every ball that hits the wheel shows up as an RPM dip (> AUTO_SHOOT_HIT_RPM below target).
 *             "Done" (no ball sensor needed) =
 *               - at least one ball went through and no new dip for AUTO_SHOOT_EMPTY_S, or
 *               - no ball at all within AUTO_SHOOT_NO_BALL_S (nothing loaded), or
 *               - AUTO_SHOOT_MAX_S safety limit
 *             -> close stopper, DONE
 *   DONE      waits for RT to be released (no re-opening while RT is still held). Release RT -> IDLE.
 *
 * The driver can always take over: pressing R1 (manual stopper) cancels auto for this RT press.
 */
public class AutoShoot {
    public enum Phase { IDLE, WAITING, SHOOTING, DONE }

    private final Flywheel flywheel;
    private final Stopper stopper;
    private final Telemetry telemetry;
    private Phase phase = Phase.IDLE;
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime sinceHit = new ElapsedTime();
    private boolean wasReady = false, inDip = false;
    private int balls = 0;
    private String lastReason = "";

    public AutoShoot(Flywheel flywheel, Stopper stopper, Telemetry telemetry) {
        this.flywheel = flywheel;
        this.stopper = stopper;
        this.telemetry = telemetry;
    }

    public Phase getPhase() { return phase; }

    /** The driver used R1: stop doing anything automatic until RT is released. */
    public void cancel() {
        if (phase == Phase.WAITING || phase == Phase.SHOOTING) {
            lastReason = "cancelled (R1)";
            phase = Phase.DONE;
        }
    }

    /**
     * Call every loop, BEFORE the stopper decides indexer/intake states.
     * @param flywheelOn the flywheel is spinning (RT held, or its short run-on)
     */
    public void update(boolean flywheelOn) {
        if (!RoboConstants.AUTO_SHOOT_ENABLED) {
            phase = Phase.IDLE;
            return;
        }
        if (!flywheelOn) {
            if (phase == Phase.SHOOTING) stopper.close(); // flywheel stopped mid-volley: close up
            phase = Phase.IDLE;
            return;
        }

        double target = flywheel.getTargetRpm();
        double worstErr = target - Math.min(flywheel.getLeftRpm(), flywheel.getRightRpm());

        switch (phase) {
            case IDLE:
                phase = Phase.WAITING;
                wasReady = false;
                break;

            case WAITING: {
                boolean ready = Math.abs(target - flywheel.getLeftRpm()) <= RoboConstants.AUTO_SHOOT_READY_RPM
                        && Math.abs(target - flywheel.getRightRpm()) <= RoboConstants.AUTO_SHOOT_READY_RPM;
                if (ready && !wasReady) readyTimer.reset();
                wasReady = ready;
                if (ready && readyTimer.seconds() >= RoboConstants.AUTO_SHOOT_READY_S) {
                    stopper.open();
                    phase = Phase.SHOOTING;
                    shootTimer.reset();
                    sinceHit.reset();
                    balls = 0;
                    inDip = false;
                    RobotLog.i("[AutoShoot] at speed (L %.0f R %.0f): stopper OPEN", flywheel.getLeftRpm(), flywheel.getRightRpm());
                }
                break;
            }

            case SHOOTING: {
                // Count balls: each dip below HIT_RPM is one ball (with a small gap so one dip isn't counted twice)
                boolean dip = worstErr > RoboConstants.AUTO_SHOOT_HIT_RPM;
                if (dip) sinceHit.reset();
                if (dip && !inDip) balls++;
                if (!dip && worstErr < RoboConstants.AUTO_SHOOT_HIT_RPM * 0.5) inDip = false;
                if (dip) inDip = true;

                String reason = null;
                if (balls > 0 && !inDip && sinceHit.seconds() >= RoboConstants.AUTO_SHOOT_EMPTY_S) reason = "empty (no more balls)";
                else if (balls == 0 && shootTimer.seconds() >= RoboConstants.AUTO_SHOOT_NO_BALL_S) reason = "no balls loaded";
                else if (shootTimer.seconds() >= RoboConstants.AUTO_SHOOT_MAX_S) reason = "time limit";
                if (reason != null) {
                    stopper.close();
                    phase = Phase.DONE;
                    lastReason = reason;
                    RobotLog.i("[AutoShoot] stopper CLOSED: %s, %d balls in %.2f s", reason, balls, shootTimer.seconds());
                }
                break;
            }

            case DONE:
                break; // wait for RT release
        }
    }

    public void addTelemetry() {
        if (!RoboConstants.AUTO_SHOOT_ENABLED) {
            telemetry.addData("Auto shoot", "OFF (AUTO_SHOOT_ENABLED)");
            return;
        }
        String s;
        switch (phase) {
            case WAITING:  s = "waiting for target RPM"; break;
            case SHOOTING: s = String.format("SHOOTING (%d balls, %.1f s)", balls, shootTimer.seconds()); break;
            case DONE:     s = "done: " + lastReason + " (release RT)"; break;
            default:       s = "ready (hold RT)"; break;
        }
        telemetry.addData("Auto shoot", s);
        telemetry.addData("Auto shoot/Balls", balls);
    }
}
