package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoboConstants;

/**
 * Pedro Pathing 3 constants (2026-11-10, upgraded from Pedro 2.0.4).
 * Tune in this order with AutoTune (http://192.168.43.1:10158, see Tuning.java):
 *   1. Mecanum Procedure    -> drivetrainConfig (motor names / directions; JoJa's come from RoboConstants)
 *   2. Pinpoint Procedure   -> localizerConfig  (copy the results into RoboConstants PINPOINT_...)
 *   3. Tests > Localization -> push forward: x goes up, push left: y goes up
 *   4. Foresight Procedure  -> foresightConfig  (paste the whole generated ForesightConfig over the one below)
 *   5. Tests > Line / Curved / Hold
 */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(RoboConstants.DRIVE_LEFT_FRONT_NAME);
                c.backLeftName.set(RoboConstants.DRIVE_LEFT_BACK_NAME);
                c.frontRightName.set(RoboConstants.DRIVE_RIGHT_FRONT_NAME);
                c.backRightName.set(RoboConstants.DRIVE_RIGHT_BACK_NAME);
                c.frontLeftDirection.set(RoboConstants.DRIVE_LEFT_FRONT_DIRECTION);
                c.backLeftDirection.set(RoboConstants.DRIVE_LEFT_BACK_DIRECTION);
                c.frontRightDirection.set(RoboConstants.DRIVE_RIGHT_FRONT_DIRECTION);
                c.backRightDirection.set(RoboConstants.DRIVE_RIGHT_BACK_DIRECTION);
                c.manualBrakeMode.set(true);
            }
    );

    // Values live in RoboConstants (PINPOINT_...) so JoJa Tele / JoJa Odometry Test use the same ones.
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set(RoboConstants.PINPOINT_NAME);
                c.podType.set(RoboConstants.PINPOINT_4_BAR_POD
                        ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                        : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
                c.xPodOffset.set(RoboConstants.PINPOINT_X_OFFSET_IN);
                c.yPodOffset.set(RoboConstants.PINPOINT_Y_OFFSET_IN);
                c.xPodDirection.set(RoboConstants.PINPOINT_X_REVERSED
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(RoboConstants.PINPOINT_Y_REVERSED
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }
    );

    // JoJa's own values: AutoTune > Foresight Procedure (2026-11-10). Loc Tests 1-3 checked good with these.
    // To re-tune: run the Foresight Procedure again and paste its Java tab over this whole block.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.319462699059634);
                Controller secondaryTranslationalForward = Controller.proportional(0.11803293110200244);
                Controller primaryTranslationalLateral = Controller.proportional(0.4171760208425413);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1541353925527421);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012297013328956781));
                c.brake.set(Controller.proportionalFeedforward(0.010452461329613263));

                c.headingFeedback.set(Controller.proportional(5.034222130215378));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07045603567446174, 0.005750233679162181));

                c.linearBrakeCoefficients.set(Matrix.diag(0.13782874246199367, 0.07665736708865928));
                c.quadraticBrakeCoefficients.set(Matrix.diag(4.932719034077427E-4, 0.0014250711561793656));

                c.maxAchievableForwardVelocity.set(77.21315484961137);
                c.maxAchievableStrafeVelocity.set(61.4213099802338);
                c.naturalForwardDeceleration.set(41.819996357543566);
                c.naturalStrafeDeceleration.set(63.06123297365159);
            }
    );

    public static Mecanum drivetrain(HardwareMap h) { return new Mecanum(h, drivetrainConfig); }

    public static PinpointLocalizer localizer(HardwareMap h) { return new PinpointLocalizer(h, localizerConfig); }

    /** The follower the Loc Tests (and future autos) use. */
    public static Follower create(HardwareMap h) {
        return new Follower(localizer(h), drivetrain(h), new Foresight(foresightConfig));
    }
}
