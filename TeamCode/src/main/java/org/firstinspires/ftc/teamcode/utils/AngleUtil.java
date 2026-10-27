package org.firstinspires.ftc.teamcode.utils;

public final class AngleUtil {
    private AngleUtil(){}

    /** Wrap any angle to (-180, 180] degrees */
    public static double wrapDeg(double deg) {
        double a = deg % 360.0;
        if (a <= -180.0) a += 360.0;
        if (a > 180.0) a -= 360.0;
        return a;
    }

    /** Smallest signed diff target - current (deg) */
    public static double shortestDiffDeg(double targetDeg, double currentDeg) {
        return wrapDeg(targetDeg - currentDeg);
    }

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static double radiansToDegrees(double valueRadian){
        return (valueRadian/(Math.PI))*180;
    }
    public static double degreesToRadians(double valueDegrees){
        return (valueDegrees/180)*Math.PI;
    }
}
