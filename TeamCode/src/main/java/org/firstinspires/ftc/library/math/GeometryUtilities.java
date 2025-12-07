package org.firstinspires.ftc.library.math;

public class GeometryUtilities {
    public static double normalizeAngle(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }
}
