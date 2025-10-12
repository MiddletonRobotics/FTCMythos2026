package org.firstinspires.ftc.library.settings;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CameraPosition {
    private final double x;
    private final double y;
    private final double z;
    private final double yaw;
    private final double pitch;
    private final double roll;

    public CameraPosition(final DistanceUnit distanceUnit, final AngleUnit angleUnit, double x, final double y, final double z, final double yaw, final double pitch, final double roll) {
        this.x = DistanceUnit.INCH.fromUnit(distanceUnit, x);
        this.y = DistanceUnit.INCH.fromUnit(distanceUnit, y);;
        this.z = DistanceUnit.INCH.fromUnit(distanceUnit, z);;
        this.yaw = AngleUnit.DEGREES.fromUnit(angleUnit, yaw);;
        this.pitch = AngleUnit.DEGREES.fromUnit(angleUnit, pitch);;
        this.roll = AngleUnit.DEGREES.fromUnit(angleUnit, roll);;
    }

    public double getX(DistanceUnit unit) {
        return unit.fromInches(x);
    }

    public double getY(DistanceUnit unit) {
        return unit.fromInches(y);
    }

    public double getZ(DistanceUnit unit) {
        return unit.fromInches(z);
    }

    public double getYaw(AngleUnit unit) {
        return unit.fromDegrees(yaw);
    }

    public double getPitch(AngleUnit unit) {
        return unit.fromDegrees(pitch);
    }

    public double getRoll(AngleUnit unit) {
        return unit.fromDegrees(roll);
    }
}
