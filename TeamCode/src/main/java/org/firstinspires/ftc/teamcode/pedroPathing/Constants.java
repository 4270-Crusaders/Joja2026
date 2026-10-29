package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.myLocalizer.MyFollowerBuilder;
import org.firstinspires.ftc.teamcode.myLocalizer.MyLocalizerConstants;

// Localization: drive-motor encoders + Control Hub IMU (DECODE's MyLocalizer). No odometry pods.
public class Constants {
    // goBILDA 5203 435 RPM: (1 + 46/17)^2 * 28 = ~384.5 ticks per motor rev
    private static final double DRIVE_TPR = (1 + (46.0 / 17)) * (1 + (46.0 / 17)) * 28;
    // JoJa: goBILDA 104 mm mecanum wheels
    private static final double WHEEL_DIAMETER_MM = 104;
    private static final double TICKS_TO_INCHES = (WHEEL_DIAMETER_MM * Math.PI / 25.4) / DRIVE_TPR;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.8); // JoJa ~15 lb

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RoboConstants.DRIVE_RIGHT_FRONT_NAME)
            .rightRearMotorName(RoboConstants.DRIVE_RIGHT_BACK_NAME)
            .leftFrontMotorName(RoboConstants.DRIVE_LEFT_FRONT_NAME)
            .leftRearMotorName(RoboConstants.DRIVE_LEFT_BACK_NAME)
            .rightFrontMotorDirection(RoboConstants.DRIVE_RIGHT_FRONT_DIRECTION)
            .rightRearMotorDirection(RoboConstants.DRIVE_RIGHT_BACK_DIRECTION)
            .leftFrontMotorDirection(RoboConstants.DRIVE_LEFT_FRONT_DIRECTION)
            .leftRearMotorDirection(RoboConstants.DRIVE_LEFT_BACK_DIRECTION);

    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart
     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    );

    public static MyLocalizerConstants myLocalizerConstants = new MyLocalizerConstants()
            .robot_Length(8)   // JoJa: front-to-back wheel centers, ~8 in
            .robot_Width(9.5)  // JoJa: left-to-right wheel centers, ~9.5 in
            .rightFrontMotorName(RoboConstants.DRIVE_RIGHT_FRONT_NAME)
            .rightRearMotorName(RoboConstants.DRIVE_RIGHT_BACK_NAME)
            .leftFrontMotorName(RoboConstants.DRIVE_LEFT_FRONT_NAME)
            .leftRearMotorName(RoboConstants.DRIVE_LEFT_BACK_NAME)
            // Encoder directions follow the motor directions (left side reversed). VERIFY with LocalizationTest
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu") // Control Hub built-in IMU
            // JoJa's Control Hub is on a side wall. PLACEHOLDER: confirm logo/USB direction
            .IMU_Orientation(new RevHubOrientationOnRobot
                    (RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .strafeTicksToInches(TICKS_TO_INCHES)   // refine with Tuning > LateralTuner
            .forwardTicksToInches(TICKS_TO_INCHES); // refine with Tuning > ForwardTuner
//            .turnTicksToRadians(0.09); //uses imu :)

    public static Follower createFollower(HardwareMap hardwareMap, Telemetry tele) {
        return new MyFollowerBuilder(followerConstants, hardwareMap)
                .myLocalizer(myLocalizerConstants, tele)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
