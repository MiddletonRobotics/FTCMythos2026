package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.library.math.geometry.Rotation2d;

public class DrivetrainConstants {
    public static final String kSubsystemName = "Drivetrain ";

    public static final String fLMotorID = "leftFront";
    public static final String fRMotorID = "rightFront";
    public static final String bLMotorID = "leftRear";
    public static final String bRMotorID = "rightRear";

    public static final double kMaximumLinearVelocityInchesPerSecond = 5.27 * 12;
    public static final double kMaximumRotationRadiansPerSecond = 2 * Math.PI;

    public static final Pose kCloseGoalStartingPoseBlue = new Pose(24.250, 130.250, Math.toRadians(144));
    public static final Pose kFarStartingPoseBlue = new Pose(56.000, 8.75, Math.toRadians(90));

    public static final Pose kAutoCloseShootingPositionBlue = new Pose(56, 84, Math.toRadians(180));
    public static final Pose kAutoClosePickupOnePositionBlue = new Pose(15, 84, Math.toRadians(180));
    public static final Pose kAutoClosePickupTwoControlPositionBlue = new Pose(59, 58, Math.toRadians(180));
    public static final Pose kAutoClosePickupTwoPositionBlue = new Pose(17, 60, Math.toRadians(180));
    public static final Pose kAutoClosePickupThreeControlPositionBlue = new Pose(62, 31, Math.toRadians(180));
    public static final Pose kAutoClosePickupThreePositionBlue = new Pose(15, 36, Math.toRadians(180));
    public static final Pose kAutoCloseParkingPositionBlue = new Pose(20, 100, Math.toRadians(90));

    public static final Pose kAutoFarShootingPositionBlue = new Pose(59.6, 18.3, Math.toRadians(180));
    public static final Pose kAutoFarPickupOnePositionBlue = new Pose(15.2, 35.9, Math.toRadians(180));
    public static final Pose kAutoFarPickupOneControlPositionBlue = new Pose(64, 37.5, Math.toRadians(180));
    public static final Pose kAutoFarPreparePickupTwoPositionBlue = new Pose(17.75, 16, Math.toRadians(200));
    public static final Pose kAutoFarPreparePickupTwoControlPositionBlue = new Pose(45.5, 35, Math.toRadians(200));
    public static final Pose kAutoFarPickupTwoPositionBlue = new Pose(9, 10.5, Math.toRadians(200));
    public static final Pose kAutoFarParkingPositionBlue = new Pose(40, 20, Math.toRadians(90));

    public static Pose decideToFlipPose(GlobalConstants.AllianceColor alliance, Pose poseToPotentiallyFlip) {
        return alliance == GlobalConstants.AllianceColor.BLUE ? poseToPotentiallyFlip : poseToPotentiallyFlip.mirror();
    }

    public static double decideToFlipHeading(GlobalConstants.AllianceColor alliance, double headingToPotentiallyFlip) {
        return alliance == GlobalConstants.AllianceColor.BLUE ? headingToPotentiallyFlip : MathFunctions.normalizeAngle(Math.PI - headingToPotentiallyFlip);
    }
}
