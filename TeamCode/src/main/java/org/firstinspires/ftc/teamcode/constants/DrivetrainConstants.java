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
    public static final Pose kFarStartingPoseBlue = new Pose(56.000, 8, Math.toRadians(90));
    public static final Pose kHumanStartingPoseBlue = new Pose(135.25, 7.5, Math.toRadians(180));

    public static final Pose kTopLeftParkingPoseBlue = new Pose(98, 26.5, Math.toRadians(315));
    public static final Pose kTopRightParkingPoseBlue = new Pose(98, 40, Math.toRadians(225));
    public static final Pose kBottomLeftParkingPoseBlue = new Pose(112.1, 26.5, Math.toRadians(45));
    public static final Pose kBottomRightParkingPoseBlue = new Pose(112.1, 40, Math.toRadians(135));

    public static final Pose kTopLeftParkingPoseRed = kTopRightParkingPoseBlue.mirror();
    public static final Pose kTopRightParkingPoseRed = kTopLeftParkingPoseBlue.mirror();
    public static final Pose kBottomLeftParkingPoseRed = kBottomRightParkingPoseBlue.mirror();
    public static final Pose kBottomRightParkingPoseRed = kBottomLeftParkingPoseBlue.mirror();

    public static final Pose kAutoCloseShootingPositionBlue = new Pose(56, 84, Math.toRadians(180));
    public static final Pose kAutoClosePickupOnePositionBlue = new Pose(15, 86, Math.toRadians(180));
    public static final Pose kAutoClosePickupTwoReadyPositionBlue = new Pose(43.5, 60, Math.toRadians(180));
    public static final Pose kAutoClosePickupTwoPositionBlue = new Pose(12, 60, Math.toRadians(180));
    public static final Pose kAutoClosePickupThreeReadyPositionBlue = new Pose(43, 36, Math.toRadians(180));
    public static final Pose kAutoClosePickupThreePositionBlue = new Pose(13, 36, Math.toRadians(180));
    public static final Pose kAutoCloseParkingPositionBlue = new Pose(20, 100, Math.toRadians(180));

    public static final Pose kAutoClosePrepareGateBlue = new Pose(26, 73, Math.toRadians(180));
    public static final Pose kAutoCloseGateBlue = new Pose(15, 73, Math.toRadians(180));

    public static final Pose kAutoFarShootingPositionBlue = new Pose(59.6, 18.3, Math.toRadians(180));
    public static final Pose kAutoFarPickupOnePositionBlue = new Pose(11, 35.5, Math.toRadians(180));
    public static final Pose kAutoFarPickupOneReadyPositionBlue = new Pose(46, 35.5, Math.toRadians(180));
    public static final Pose kAutoFarPickupTwoReadyPositionBlue = new Pose(22, 10, Math.toRadians(180));
    public static final Pose kAutoFarPickupTwoPositionBlue = new Pose(8, 10, Math.toRadians(180));
    public static final Pose kAutoFarReadyThreePositionBlue = new Pose(12, 56, Math.toRadians(225));
    public static final Pose kAutoFarPickupThreePositionBlue = new Pose(12, 10, Math.toRadians(225));
    public static final Pose kAutoFarParkingPositionBlue = new Pose(40, 20, Math.toRadians(180));

    public static Pose decideToFlipPose(GlobalConstants.AllianceColor alliance, Pose poseToPotentiallyFlip) {
        return alliance == GlobalConstants.AllianceColor.BLUE ? poseToPotentiallyFlip : poseToPotentiallyFlip.mirror();
    }

    public static double decideToFlipHeading(GlobalConstants.AllianceColor alliance, double headingToPotentiallyFlip) {
        return alliance == GlobalConstants.AllianceColor.BLUE ? headingToPotentiallyFlip : MathFunctions.normalizeAngle(Math.PI - headingToPotentiallyFlip);
    }
}
