package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Match loader servo test: move the servo by hand-picked steps to find the zero / UP / MIDDLE / DOWN positions.
 * Raw servo position 0.0 .. 1.0 (no offset added). The servo does NOT move until you press something.
 *
 *   D-pad up / down      +0.05 / -0.05   (big steps)
 *   D-pad right / left   +0.01 / -0.01
 *   RB / LB              +0.002 / -0.002 (fine)
 *   A  = 0.0 (servo zero)     B = 0.5 (servo center)
 *   X  = go to the saved UP -> MIDDLE -> DOWN position (cycles), to check the current RoboConstants values
 *   Y  = servo OFF (limp, no power) so you can move / re-mount the horn by hand. Any move button turns it back on.
 *   Hold START + press:  D-pad up = save as UP,  D-pad right = save as MIDDLE,  D-pad down = save as DOWN
 *       (live until the app restarts; copy the numbers shown into RoboConstants to keep them)
 *
 * CAREFUL: 0.0 / 1.0 are the ends of the servo's travel. If the plate hits something there, the servo stalls:
 * use small steps near the ends.
 */
@TeleOp(name="JoJa Match Loader Test", group="Test")
public class JoJaMatchLoaderTest extends OpMode {
    private TelemetryManager panels;
    private Telemetry tele;
    private Servo servo;
    private double pos = Double.NaN;  // NaN = not commanded yet
    private boolean powered = false;
    private int cycle = 0;            // X: 0 = UP, 1 = MIDDLE, 2 = DOWN
    private String lastAction = "nothing yet (servo not moved)";
    private boolean pUp, pDown, pLeft, pRight, pRb, pLb, pA, pB, pX, pY;

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        tele = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        servo = hardwareMap.get(Servo.class, RoboConstants.MATCH_LOADER_SERVO_NAME);
        tele.addLine("JoJa Match Loader Test. Servo won't move until you press a button after Play.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        boolean save = gamepad1.start;

        if (save) {
            if (pressed(gamepad1.dpad_up, pUp) && !Double.isNaN(pos))    { RoboConstants.MATCH_LOADER_UP_POS = pos;     lastAction = String.format("saved UP = %.3f", pos); }
            if (pressed(gamepad1.dpad_right, pRight) && !Double.isNaN(pos)) { RoboConstants.MATCH_LOADER_MIDDLE_POS = pos; lastAction = String.format("saved MIDDLE = %.3f", pos); }
            if (pressed(gamepad1.dpad_down, pDown) && !Double.isNaN(pos))  { RoboConstants.MATCH_LOADER_DOWN_POS = pos;   lastAction = String.format("saved DOWN = %.3f", pos); }
        } else {
            double start = Double.isNaN(pos) ? 0.5 : pos;
            if (pressed(gamepad1.dpad_up, pUp))       move(start + 0.05, "+0.05");
            if (pressed(gamepad1.dpad_down, pDown))   move(start - 0.05, "-0.05");
            if (pressed(gamepad1.dpad_right, pRight)) move(start + 0.01, "+0.01");
            if (pressed(gamepad1.dpad_left, pLeft))   move(start - 0.01, "-0.01");
            if (pressed(gamepad1.right_bumper, pRb))  move(start + 0.002, "+0.002");
            if (pressed(gamepad1.left_bumper, pLb))   move(start - 0.002, "-0.002");
            if (pressed(gamepad1.a, pA))              move(0.0, "ZERO (0.0)");
            if (pressed(gamepad1.b, pB))              move(0.5, "CENTER (0.5)");
            if (pressed(gamepad1.x, pX)) {
                double[] saved = {RoboConstants.MATCH_LOADER_UP_POS, RoboConstants.MATCH_LOADER_MIDDLE_POS, RoboConstants.MATCH_LOADER_DOWN_POS};
                String[] names = {"saved UP", "saved MIDDLE", "saved DOWN"};
                move(saved[cycle], names[cycle]);
                cycle = (cycle + 1) % 3;
            }
            if (pressed(gamepad1.y, pY)) setPowered(false, "servo OFF (limp)");
        }
        pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down; pLeft = gamepad1.dpad_left; pRight = gamepad1.dpad_right;
        pRb = gamepad1.right_bumper; pLb = gamepad1.left_bumper; pA = gamepad1.a; pB = gamepad1.b; pX = gamepad1.x; pY = gamepad1.y;

        tele.addData("Servo position", Double.isNaN(pos) ? "not moved yet" : String.format("%.3f", pos));
        tele.addData("Servo angle (approx)", Double.isNaN(pos) ? "-" : String.format("%.0f deg of ~270", pos * 270.0));
        tele.addData("Servo power", powered ? "ON (holding)" : "OFF (limp)");
        tele.addData("Last", lastAction);
        tele.addLine("");
        tele.addData("RoboConstants UP / MIDDLE / DOWN", "%.3f / %.3f / %.3f",
                RoboConstants.MATCH_LOADER_UP_POS, RoboConstants.MATCH_LOADER_MIDDLE_POS, RoboConstants.MATCH_LOADER_DOWN_POS);
        tele.addData("RoboConstants OFFSET", "%.3f (TeleOp adds this to all three)", RoboConstants.MATCH_LOADER_OFFSET);
        tele.addLine("");
        tele.addLine("D-pad U/D +-0.05   D-pad R/L +-0.01   RB/LB +-0.002");
        tele.addLine("A = 0.0   B = 0.5   X = saved UP/MID/DOWN   Y = servo limp");
        tele.addLine("START + D-pad up/right/down = save as UP / MIDDLE / DOWN");
        panels.update(telemetry);
    }

    private void move(double p, String what) {
        pos = Math.max(0.0, Math.min(1.0, p));
        setPowered(true, null);
        servo.setPosition(pos);
        lastAction = String.format("%s -> %.3f", what, pos);
    }

    private void setPowered(boolean on, String what) {
        if (servo instanceof PwmControl) {
            if (on) ((PwmControl) servo).setPwmEnable();
            else ((PwmControl) servo).setPwmDisable();
        }
        powered = on;
        if (what != null) lastAction = what;
    }

    private static boolean pressed(boolean now, boolean before) { return now && !before; }
}
