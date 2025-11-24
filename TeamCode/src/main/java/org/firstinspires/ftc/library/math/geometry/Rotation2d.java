package org.firstinspires.ftc.library.math.geometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A rotation in a 2D coordinate frame represented by a point on the unit circle (cosine and sine).
 *
 * <p>The angle is continuous, that is if a Rotation2d is constructed with 361 degrees, it will
 * return 361 degrees. This allows algorithms that wouldn't want to see a discontinuity in the
 * rotations as it sweeps past from 360 to 0 on the second time around.
 */

public class Rotation2d {
    private final double value;
    private final double cos;
    private final double sin;

    public final static Rotation2d kPi = new Rotation2d(Math.PI);

    /**
     * Constructs a Rotation2d with a default angle of 0 degrees.
     */
    public Rotation2d() {
        this.value = 0.0;
        this.cos = 1.0;
        this.sin = 0.0;
    }

    /**
     * Constructs a Rotation2d with the given radian value.
     * The x and y don't have to be normalized.
     *
     * @param value The value of the angle in radians.
     */
    public Rotation2d(double value) {
        while (value > Math.PI) value -= 2 * Math.PI;
        while (value < -Math.PI) value += 2 * Math.PI;
        this.value = value;
        cos = Math.cos(value);
        sin = Math.sin(value);
    }

    /**
     * Constructs a Rotation2d with the given x and y (cosine and sine)
     * components.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sine of the rotation.
     */
    @SuppressWarnings("ParameterName")
    public Rotation2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-6) {
            sin = y / magnitude;
            cos = x / magnitude;
        } else {
            sin = 0.0;
            cos = 1.0;
        }

        value = Math.atan2(sin, cos);
    }

    /**
     * Constructs and returns a Rotation2d with the given degree value.
     *
     * @param degrees The value of the angle in degrees.
     * @return The rotation object with the desired angle value.
     */
    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Math.toRadians(degrees));
    }

    public static Rotation2d fromRadians(double radians) {
        return new Rotation2d(radians);
    }

    /**
     * Adds two rotations together, with the result being bounded between -pi and
     * pi.
     *
     * <p>For example, Rotation2d.fromDegrees(30) + Rotation2d.fromDegrees(60) =
     * Rotation2d{-pi/2}
     *
     * @param other The rotation to add.
     * @return The sum of the two rotations.
     */
    public Rotation2d plus(Rotation2d other) {
        return rotateBy(other);
    }

    /**
     * Subtracts the new rotation from the current rotation and returns the new
     * rotation.
     *
     * <p>For example, Rotation2d.fromDegrees(10) - Rotation2d.fromDegrees(100) =
     * Rotation2d{-pi/2}
     *
     * @param other The rotation to subtract.
     * @return The difference between the two rotations.
     */
    public Rotation2d minus(Rotation2d other) {
        return rotateBy(other.unaryMinus());
    }

    /**
     * Takes the inverse of the current rotation. This is simply the negative of
     * the current angular value.
     *
     * @return The inverse of the current rotation.
     */
    public Rotation2d unaryMinus() {
        return new Rotation2d(-value);
    }

    /**
     * Multiplies the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Rotation2d.
     */
    public Rotation2d times(double scalar) {
        return new Rotation2d(value * scalar);
    }

    /**
     * Adds the new rotation to the current rotation using a rotation matrix.
     *
     * <p>The matrix multiplication is as follows:
     * [cos_new]   [other.cos, -other.sin][cos]
     * [sin_new] = [other.sin,  other.cos][sin]
     * value_new = atan2(cos_new, sin_new)
     *
     * @param other The rotation to rotate by.
     * @return The new rotated Rotation2d.
     */
    public Rotation2d rotateBy(Rotation2d other) {
        return new Rotation2d(
                cos * other.cos - sin * other.sin,
                cos * other.sin + sin * other.cos
        );
    }

    /*
     * Returns the radian value of the rotation.
     *
     * @return The radian value of the rotation.
     */
    public double getRadians() {
        return value;
    }

    /**
     * Returns the degree value of the rotation.
     *
     * @return The degree value of the rotation.
     */
    public double getDegrees() {
        return Math.toDegrees(value);
    }

    /**
     * Returns the angle value of the rotation in the specified angle unit.
     *
     * @param angleUnit The angle unit of the value to be returned.
     * @return The value of the rotation.
     */
    public double getAngle(AngleUnit angleUnit) {
        return angleUnit == AngleUnit.RADIANS ? getRadians() : getDegrees();
    };

    /**
     * Returns the cosine of the rotation.
     *
     * @return The cosine of the rotation.
     */
    public double getCos() {
        return cos;
    }

    /**
     * Returns the sine of the rotation.
     *
     * @return The sine of the rotation.
     */
    public double getSin() {
        return sin;
    }

    /**
     * Returns the tangent of the rotation.
     *
     * @return The tangent of the rotation.
     */
    public double getTan() {
        return sin / cos;
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(Rads: %.2f, Deg: %.2f)", value, Math.toDegrees(value));
    }

    /**
     * Checks equality between this Rotation2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Rotation2d) {
            return Math.abs(((Rotation2d) obj).value - value) < 1E-9;
        }
        return false;
    }

}