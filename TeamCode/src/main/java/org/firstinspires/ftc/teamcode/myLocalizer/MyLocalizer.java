package org.firstinspires.ftc.teamcode.myLocalizer;

import com.pedropathing.ftc.localization.CustomIMU;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyLocalizer implements Localizer{
    private final Telemetry tele; // used by the debug lines in update(); uncomment them to see raw encoder deltas

    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private Matrix prevRotationMatrix;
    private final NanoTimer timer;
    private long deltaTimeNano;
    public final CustomIMU imu;
    private double previousIMUOrientation;
    private double deltaRadians;
    private double totalHeading;
    private final Encoder leftFront;
    private final Encoder rightFront;
    private final Encoder leftRear;
    private final Encoder rightRear;
    public static double FORWARD_TICKS_TO_INCHES;
    public static double STRAFE_TICKS_TO_INCHES;
    public static double TURN_TICKS_TO_RADIANS;
    public static double ROBOT_WIDTH;
    public static double ROBOT_LENGTH;

    public static boolean useIMU = true;


    /**
     * This creates a new DriveEncoderLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public MyLocalizer(HardwareMap map, MyLocalizerConstants constants, Telemetry tele) {
        this(map, constants, new Pose(), tele);
    }

    /**
     * This creates a new DriveEncoderLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public MyLocalizer(HardwareMap map, MyLocalizerConstants constants, Pose setStartPose, Telemetry tele) {
        this.tele = tele;
        FORWARD_TICKS_TO_INCHES = constants.forwardTicksToInches;
        STRAFE_TICKS_TO_INCHES = constants.strafeTicksToInches;

        // FIX: use the proper constant for turning conversion (radians per tick)
        TURN_TICKS_TO_RADIANS = constants.turnTicksToRadians;

        imu = constants.imu;
        imu.initialize(map, constants.IMU_HardwareMapName, constants.IMU_Orientation);

        ROBOT_WIDTH = constants.robot_Width;
        ROBOT_LENGTH = constants.robot_Length;

        leftFront = new Encoder(map.get(DcMotorEx.class, constants.leftFrontMotorName));
        leftRear = new Encoder(map.get(DcMotorEx.class, constants.leftRearMotorName));
        rightRear = new Encoder(map.get(DcMotorEx.class, constants.rightRearMotorName));
        rightFront = new Encoder(map.get(DcMotorEx.class, constants.rightFrontMotorName));

        leftFront.setDirection(constants.leftFrontEncoderDirection);
        leftRear.setDirection(constants.leftRearEncoderDirection);
        rightFront.setDirection(constants.rightFrontEncoderDirection);
        rightRear.setDirection(constants.rightRearEncoderDirection);

        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose = new Pose();
        currentVelocity = new Pose();
        totalHeading = 0;

        // Seed previousIMUOrientation to current imu heading so first delta is small/valid
        previousIMUOrientation = MathFunctions.normalizeAngle(imu.getHeading());

        resetEncoders();
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return startPose.plus(displacementPose);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * This sets the Matrix that contains the previous pose's heading rotation.
     *
     * @param heading the rotation of the Matrix
     */
    public void setPrevRotationMatrix(double heading) {
        prevRotationMatrix = new Matrix(3,3);
        prevRotationMatrix.set(0, 0, Math.cos(heading));
        prevRotationMatrix.set(0, 1, -Math.sin(heading));
        prevRotationMatrix.set(1, 0, Math.sin(heading));
        prevRotationMatrix.set(1, 1, Math.cos(heading));
        prevRotationMatrix.set(2, 2, 1.0);
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        displacementPose = setPose.minus(startPose);
        resetEncoders();
    }

    /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders. Then, the robot's global change in position is calculated
     * using the pose exponential method.
     */
    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas;
        setPrevRotationMatrix(getPose().getHeading());

        Matrix transformation = new Matrix(3,3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), robotDeltas);

        displacementPose = displacementPose.plus(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        currentVelocity = new Pose(globalDeltas.get(0, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano / Math.pow(10.0, 9)));

        totalHeading += globalDeltas.get(2, 0);

//        tele.addData("LeftFront",leftFront.getDeltaPosition());
//        tele.addData("LeftBack",leftRear.getDeltaPosition());
//        tele.addData("RightFront",rightFront.getDeltaPosition());
//        tele.addData("RightBack",rightRear.getDeltaPosition());
    }

    /**
     * This updates the Encoders.
     */
    public void updateEncoders() {
        leftFront.update();
        rightFront.update();
        leftRear.update();
        rightRear.update();

        double currentIMUOrientation = MathFunctions.normalizeAngle(imu.getHeading());
        deltaRadians = MathFunctions.getTurnDirection(previousIMUOrientation, currentIMUOrientation)
                * MathFunctions.getSmallestAngleDifference(currentIMUOrientation, previousIMUOrientation);
        previousIMUOrientation = currentIMUOrientation;

    }

    /**
     * This resets the Encoders.
     */
    public void resetEncoders() {
        leftFront.reset();
        rightFront.reset();
        leftRear.reset();
        rightRear.reset();
    }

    /**
     * This calculates the change in position from the perspective of the robot using information
     * from the Encoders.
     *
     * @return returns a Matrix containing the robot relative movement.
     */
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        // Average of the four wheel forward contributions (divide by 4)
        double forwardSum = leftFront.getDeltaPosition() + rightFront.getDeltaPosition()
                + leftRear.getDeltaPosition() + rightRear.getDeltaPosition();
        returnMatrix.set(0,0, FORWARD_TICKS_TO_INCHES * (forwardSum / 4.0));

        //y/strafe movement
        // Average of the four wheel strafe contributions (divide by 4)
        double strafeSum = leftFront.getDeltaPosition() - rightFront.getDeltaPosition()
                - leftRear.getDeltaPosition() + rightRear.getDeltaPosition();
        returnMatrix.set(1,0, STRAFE_TICKS_TO_INCHES * (strafeSum / 4.0));

        // theta/turning
        double imuThreshold = 1e-5; // radians; tune if necessary
        if (useIMU && Math.abs(deltaRadians) > imuThreshold) {
            returnMatrix.set(2, 0, deltaRadians);
        } else {
            // encoder-based turning contribution: average of wheel rotation diffs, normalized by robot geometry
            double turnSum = -leftFront.getDeltaPosition() + rightFront.getDeltaPosition()
                    - leftRear.getDeltaPosition() + rightRear.getDeltaPosition();
            returnMatrix.set(2,0, TURN_TICKS_TO_RADIANS * (turnSum / 4.0) / (ROBOT_WIDTH + ROBOT_LENGTH));
        }

        return returnMatrix;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return FORWARD_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return STRAFE_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return TURN_TICKS_TO_RADIANS;
    }

    /**
     * This resets the IMU
     */
    public void resetIMU() {
        imu.resetYaw();
    }

    /**
     * This returns the IMU.
     * @return the IMU
     */
    @Override
    public double getIMUHeading() {
        return imu.getHeading();
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

}
