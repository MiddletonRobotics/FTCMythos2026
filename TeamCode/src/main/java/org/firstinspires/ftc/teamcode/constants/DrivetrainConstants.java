package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

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

    public static final Pose kCloseStartingPoseBlue = new Pose(25.500, 130.500, Math.toRadians(144));
    public static final Pose kFarStartingPoseBlue = new Pose(56.000, 6.000, Math.toRadians(90));

    public static final Pose kAutoCloseShootingPositionBlue = new Pose(57, 87, Math.toRadians(135));

    public static final Pose kAutoFarShootingPositionBlue = new Pose(61.000, 23.000, Math.toRadians(110));
    public static final Pose kAutoFarPickupPositionBlue = new Pose(15.000, 36.000, Math.toRadians(180));
    public static final Pose kAutoFarPickupControlPositionBlue = new Pose(66.000, 40.000, Math.toRadians(180));
    public static final Pose kAutoFarParkingPositionBlue = new Pose(38.000, 14.000, Math.toRadians(180));

    public static Pose decideToFlipPose(GlobalConstants.AllianceColor alliance, Pose poseToPotentiallyFlip) {
        return alliance == GlobalConstants.AllianceColor.BLUE ? poseToPotentiallyFlip : poseToPotentiallyFlip.mirror();
    }
}
