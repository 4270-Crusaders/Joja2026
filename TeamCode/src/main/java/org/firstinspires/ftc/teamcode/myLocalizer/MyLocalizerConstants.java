package org.firstinspires.ftc.teamcode.myLocalizer;

import com.pedropathing.ftc.localization.CustomIMU;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.RevHubIMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class MyLocalizerConstants {
    public double forwardTicksToInches = 0.09;
    public double strafeTicksToInches = 0.09;
    public double turnTicksToRadians = 0.5;
    public double robot_Width = 1;
    public double robot_Length = 1;
    public double leftFrontEncoderDirection = Encoder.FORWARD;
    public double leftRearEncoderDirection = Encoder.FORWARD;
    public double rightFrontEncoderDirection = Encoder.FORWARD;
    public double rightRearEncoderDirection = Encoder.FORWARD;
    public String leftFrontMotorName = "leftFront";
    public String leftRearMotorName = "leftRear";
    public String rightFrontMotorName = "rightFront";
    public String rightRearMotorName = "rightRear";
    public String IMU_HardwareMapName = "imu";
    public RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
    public CustomIMU imu = new RevHubIMU();

    /**
     * This creates a new ThreeWheelIMUConstants with default values.
     */
    public MyLocalizerConstants() {
        defaults();
    }

    public MyLocalizerConstants forwardTicksToInches(double forwardTicksToInches) {
        this.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    public MyLocalizerConstants strafeTicksToInches(double strafeTicksToInches) {
        this.strafeTicksToInches = strafeTicksToInches;
        return this;
    }

    public MyLocalizerConstants turnTicksToRadians(double turnTicksToRadians) {
        this.turnTicksToRadians = turnTicksToRadians;
        return this;
    }

    public MyLocalizerConstants robot_Width(double robot_Width) {
        this.robot_Width = robot_Width;
        return this;
    }

    public MyLocalizerConstants robot_Length(double robot_Length) {
        this.robot_Length = robot_Length;
        return this;
    }

    public MyLocalizerConstants leftFrontEncoderDirection(double leftFrontEncoderDirection) {
        this.leftFrontEncoderDirection = leftFrontEncoderDirection;
        return this;
    }

    public MyLocalizerConstants rightFrontEncoderDirection(double rightFrontEncoderDirection) {
        this.rightFrontEncoderDirection = rightFrontEncoderDirection;
        return this;
    }

    public MyLocalizerConstants leftRearEncoderDirection(double leftRearEncoderDirection) {
        this.leftRearEncoderDirection = leftRearEncoderDirection;
        return this;
    }

    public MyLocalizerConstants rightRearEncoderDirection(double rightRearEncoderDirection) {
        this.rightRearEncoderDirection = rightRearEncoderDirection;
        return this;
    }

    public MyLocalizerConstants leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public MyLocalizerConstants leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public MyLocalizerConstants rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public MyLocalizerConstants rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public MyLocalizerConstants IMU_HardwareMapName(String IMU_HardwareMapName) {
        this.IMU_HardwareMapName = IMU_HardwareMapName;
        return this;
    }

    public MyLocalizerConstants IMU_Orientation(RevHubOrientationOnRobot IMU_Orientation) {
        this.IMU_Orientation = IMU_Orientation;
        return this;
    }
    public MyLocalizerConstants customIMU(CustomIMU customIMU) {
        this.imu = customIMU;
        return this;
    }

    public void defaults() {
        forwardTicksToInches = 1;
        strafeTicksToInches = 1;
        turnTicksToRadians = 1;

        robot_Width = 1;
        robot_Length = 1;

        leftFrontEncoderDirection = Encoder.FORWARD;
        leftRearEncoderDirection = Encoder.FORWARD;
        rightFrontEncoderDirection = Encoder.FORWARD;
        rightRearEncoderDirection = Encoder.FORWARD;

        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftRear";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightRear";

        IMU_HardwareMapName = "imu";
        IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu = new RevHubIMU();
    }
}
