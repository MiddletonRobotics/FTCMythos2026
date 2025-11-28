package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.library.math.geometry.Rotation2d;

public class DrivetrainConstants {
    public static final String fLMotorID = "leftFront";
    public static final String fRMotorID = "rightFront";
    public static final String bLMotorID = "leftRear";
    public static final String bRMotorID = "rightRear";

    public static final double kMaximumLinearVelocityInchesPerSecond = 5.27 * 12;
    public static final double kMaximumRotationRadiansPerSecond = 2 * Math.PI;

    public static final double kDriveToPointStaticFriction = 0.003;

    public static final Pose kCloseStartingPoseBlue = new Pose(7.2, 6.14, Math.toRadians(135));
    public static final Pose kFarStartingPoseBlue = new Pose(56, 8, Math.toRadians(90));

    public static final Pose kAutoCloseShootingPositionBlue = new Pose(57, 87, Math.toRadians(135));

    public static Pose decideToFlipPose(GlobalConstants.AllianceColor alliance, Pose poseToPotentiallyFlip) {
        return alliance == GlobalConstants.AllianceColor.BLUE ? poseToPotentiallyFlip : poseToPotentiallyFlip.mirror();
    }
}
