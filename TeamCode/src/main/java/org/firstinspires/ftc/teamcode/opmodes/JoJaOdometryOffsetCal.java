package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Measures the Pinpoint pod OFFSETS automatically (PINPOINT_X_OFFSET_IN / PINPOINT_Y_OFFSET_IN).
 *
 * Why: with wrong offsets, turning in place makes the pods look like the robot is DRIVING, so Pedro "corrects"
 * that fake motion and the robot wanders while it turns (Loc Test 3).
 *
 * How: the robot spins in place TURNS times counterclockwise, then TURNS times clockwise, with offsets set to 0.
 * When a robot turns in place, each pod rolls a distance = (its distance from the turning center) x (angle turned).
 * So offset = pod distance / angle. Both directions are measured and averaged (cancels wheel slip).
 *
 * Needs the pod DIRECTIONS right first (JoJa Odometry Test: push forward -> X up, push left -> Y up).
 * Put the robot on the field tiles with ~2 ft of room around it.
 *
 * A = start.  B = stop.  Y = apply the result live (until restart). Copy the numbers into RoboConstants.
 */
@TeleOp(name="JoJa Odometry Offset Cal", group="Test")
public class JoJaOdometryOffsetCal extends OpMode {
    private static final double TURNS = 3.0;
    private static final double TURN_POWER = 0.3;    // slow = less wheel slip
    private static final double RAMP_SECONDS = 0.5;

    private enum Phase { READY, CCW, PAUSE, CW, DONE }

    private TelemetryManager panels;
    private Telemetry tele;
    private GoBildaPinpointDriver pinpoint;
    private Drivetrain drivetrain;
    private Phase phase = Phase.READY;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private boolean lastY = false, applied = false;

    // Per direction: pod travel in ROBOT frame (inches) and total angle turned (radians)
    private final double[] sumFwd = new double[2], sumLeft = new double[2], sumAngle = new double[2];
    private double prevX, prevY, prevH;
    private double resultX = Double.NaN, resultY = Double.NaN;      // X offset (fwd pod left +), Y offset (strafe pod fwd +)
    private final double[] estX = new double[2], estY = new double[2];

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RoboConstants.PINPOINT_NAME);
        pinpoint.setEncoderResolution(RoboConstants.PINPOINT_4_BAR_POD
                ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                RoboConstants.PINPOINT_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                RoboConstants.PINPOINT_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.INCH); // measure the raw pod motion
        pinpoint.resetPosAndIMU();
        drivetrain = new Drivetrain(hardwareMap,
                RoboConstants.DRIVE_LEFT_FRONT_NAME, RoboConstants.DRIVE_LEFT_BACK_NAME,
                RoboConstants.DRIVE_RIGHT_FRONT_NAME, RoboConstants.DRIVE_RIGHT_BACK_NAME, tele);
        tele.addLine("JoJa Odometry Offset Cal. Robot on the floor, ~2 ft clear all around. Keep still, press Play.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double h = pinpoint.getHeading(AngleUnit.RADIANS);

        if (gamepad1.b && phase != Phase.DONE && phase != Phase.READY) {
            phase = Phase.READY;
        }

        int dir = (phase == Phase.CCW) ? 0 : (phase == Phase.CW) ? 1 : -1;
        if (dir >= 0) {
            // Field motion since last loop -> robot frame (using the heading halfway through the step)
            double dh = wrap(h - prevH);
            double mid = prevH + dh / 2.0;
            double dx = x - prevX, dy = y - prevY;
            sumFwd[dir]  +=  Math.cos(mid) * dx + Math.sin(mid) * dy;
            sumLeft[dir] += -Math.sin(mid) * dx + Math.cos(mid) * dy;
            sumAngle[dir] += dh;
        }
        prevX = x; prevY = y; prevH = h;

        double turn = 0.0; // Drivetrain: turn + = clockwise
        switch (phase) {
            case READY:
                if (gamepad1.a) { zero(); phase = Phase.CCW; phaseTimer.reset(); }
                break;
            case CCW:
                turn = -ramp();
                if (Math.abs(sumAngle[0]) >= TURNS * 2 * Math.PI) { phase = Phase.PAUSE; phaseTimer.reset(); }
                break;
            case PAUSE:
                if (phaseTimer.seconds() > 0.75) { phase = Phase.CW; phaseTimer.reset(); }
                break;
            case CW:
                turn = ramp();
                if (Math.abs(sumAngle[1]) >= TURNS * 2 * Math.PI) { phase = Phase.DONE; compute(); }
                break;
            case DONE:
                if (gamepad1.y && !lastY && !Double.isNaN(resultX)) {
                    RoboConstants.PINPOINT_X_OFFSET_IN = resultX;
                    RoboConstants.PINPOINT_Y_OFFSET_IN = resultY;
                    applied = true;
                }
                if (gamepad1.a) { zero(); phase = Phase.CCW; phaseTimer.reset(); applied = false; }
                break;
        }
        lastY = gamepad1.y;
        drivetrain.drive(0.0, 0.0, turn);
        drivetrain.periodic();

        tele.addData("Phase", phase);
        tele.addData("Turned CCW / CW", "%.0f / %.0f deg", Math.toDegrees(sumAngle[0]), Math.toDegrees(sumAngle[1]));
        tele.addData("Pose (offsets 0)", "x %.2f  y %.2f  h %.1f", x, y, Math.toDegrees(h));
        if (phase == Phase.READY) tele.addLine("A = start (spins 3 turns left, then 3 turns right). B = stop.");
        if (phase == Phase.DONE) {
            tele.addData("CCW says X / Y offset", "%.2f / %.2f in", estX[0], estY[0]);
            tele.addData("CW  says X / Y offset", "%.2f / %.2f in", estX[1], estY[1]);
            tele.addData("PINPOINT_X_OFFSET_IN", "%.2f   (forward pod: left + / right -)", resultX);
            tele.addData("PINPOINT_Y_OFFSET_IN", "%.2f   (strafe pod: forward + / back -)", resultY);
            if (Math.abs(estX[0] - estX[1]) > 0.5 || Math.abs(estY[0] - estY[1]) > 0.5) {
                tele.addLine("CCW and CW disagree by > 0.5 in: wheels slipped or a pod direction is wrong. Run again (A).");
            }
            tele.addLine(applied ? "APPLIED live. Copy both numbers into RoboConstants to keep them."
                    : "Y = apply live. A = run again.");
        }
        panels.update(telemetry);
    }

    /** Smooth start so the wheels don't slip. */
    private double ramp() {
        return TURN_POWER * Math.min(1.0, phaseTimer.seconds() / RAMP_SECONDS);
    }

    private void zero() {
        for (int i = 0; i < 2; i++) { sumFwd[i] = 0; sumLeft[i] = 0; sumAngle[i] = 0; }
        resultX = resultY = Double.NaN;
    }

    /**
     * Turning in place by angle a (CCW +): a pod at (forward f, left l) from the center rolls
     * forward pod: -l * a        strafe pod: +f * a
     * so  X offset (forward pod's left distance)  = -fwd / a,   Y offset (strafe pod's forward distance) = left / a.
     */
    private void compute() {
        for (int i = 0; i < 2; i++) {
            estX[i] = -sumFwd[i] / sumAngle[i];
            estY[i] = sumLeft[i] / sumAngle[i];
        }
        resultX = 0.5 * (estX[0] + estX[1]);
        resultY = 0.5 * (estY[0] + estY[1]);
        RobotLog.i(String.format("[OffsetCal] CCW X=%.3f Y=%.3f  CW X=%.3f Y=%.3f  -> X=%.3f Y=%.3f in",
                estX[0], estY[0], estX[1], estY[1], resultX, resultY));
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    @Override
    public void stop() {
        drivetrain.stop();
        drivetrain.periodic();
    }
}
