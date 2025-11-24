package org.firstinspires.ftc.library.math.geometry;

/** Represents a transformation for a Pose2d in the pose's frame. */
public class Transform2d {
    private final Translation2d translation;
    private final Rotation2d rotation;

    /**
     * Constructs the transform that maps the initial pose to the final pose.
     *
     * @param initial The initial pose for the transformation.
     * @param last    The final pose for the transformation.
     */

    public Transform2d(Pose2d initial, Pose2d last) {
        // We are rotating the difference between the translations using a clockwise rotation matrix. This transforms the global delta into a local delta (relative to the initial pose).
        translation = last.getTranslation().minus(initial.getTranslation()).rotateBy(initial.getRotation().unaryMinus());
        rotation = last.getRotation().minus(initial.getRotation());
    }

    /**
     * Constructs a transform with the given translation and rotation components.
     *
     * @param translation Translational component of the transform.
     * @param rotation    Rotational component of the transform.
     */

    public Transform2d(Translation2d translation, Rotation2d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    /**
     * Constructs the identity transform -- maps an initial pose to itself.
     */

    public Transform2d() {
        translation = new Translation2d();
        rotation = new Rotation2d();
    }

    /**
     * Scales the transform by the scalar.
     *
     * @param scalar The scalar.
     * @return The scaled Transform2d.
     */

    public Transform2d times(double scalar) {
        return new Transform2d(translation.times(scalar), rotation.times(scalar));
    }

    /**
     * Returns the translation component of the transformation.
     *
     * @return The translational component of the transform.
     */

    public Translation2d getTranslation() {
        return translation;
    }

    /**
     * Returns the rotational component of the transformation.
     *
     * @return Reference to the rotational component of the transform.
     */

    public Rotation2d getRotation() {
        return rotation;
    }

    @Override
    public String toString() {
        return String.format("Transform2d(%s, %s)", translation, rotation);
    }

    /**
     * Checks equality between this Transform2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Transform2d) {
            return ((Transform2d) obj).translation.equals(translation) && ((Transform2d) obj).rotation.equals(rotation);
        }
        
        return false;
    }

    /**
     * Invert the transformation. This is useful for undoing a transformation.
     *
     * @return The inverted transformation.
     */

    public Transform2d inverse() {
        // We are rotating the difference between the translations using a clockwise rotation matrix. This transforms the global delta into a local delta (relative to the initial pose).
        return new Transform2d(getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()), getRotation().unaryMinus());
    }


}