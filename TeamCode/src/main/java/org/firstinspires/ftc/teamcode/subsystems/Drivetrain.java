package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.KalmanFilter;
import org.firstinspires.ftc.library.controller.KalmanFilterParameters;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.library.math.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Drawing;

import java.util.List;

import lombok.Getter;
import lombok.Setter;

public class Drivetrain extends SubsystemBase {
    private IMU imu;
    private static Follower follower;
    private List<LynxModule> allHubs;

    private KalmanFilter xFilter;
    private KalmanFilter yFilter;
    private KalmanFilter headingFilter;

    @Getter @Setter
    private boolean isRobotCentric = false;

    @Setter @Getter
    private Pose drivetrainParkingPose = DrivetrainConstants.kTopLeftParkingPoseBlue;

    KalmanFilterParameters filterParameters = new KalmanFilterParameters(
            0.01,  // modelCovariance: how much you trust your model (lower = trust model more)
            0.1    // dataCovariance: how noisy your vision data is (lower = trust vision more)
    );

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    public Drivetrain(HardwareMap hMap, TelemetryManager telemetryM) {
        follower = Constants.createFollower(hMap);

        xFilter = new KalmanFilter(filterParameters);
        yFilter = new KalmanFilter(filterParameters);
        headingFilter = new KalmanFilter(filterParameters);

        allHubs = hMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        this.telemetryM = telemetryM;
        poseHistory = follower.getPoseHistory();

        initializeImu(hMap);
        drawCurrent();
    }

    public void initializeImu(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));

        imu.initialize(parameters);
    }

    @Override
    public void periodic() {
        telemetryM.addData(DrivetrainConstants.kSubsystemName + "Pose X", getPose().getX());
        telemetryM.addData(DrivetrainConstants.kSubsystemName + "Pose Y", getPose().getY());
        telemetryM.addData(DrivetrainConstants.kSubsystemName + "Pose θ", getPose().getRotation().getDegrees());

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        drawCurrentAndHistory();
    }

    public void updateWithVision(Pose2d estimatedVisionPose) {
        xFilter.update(0, estimatedVisionPose.getX());
        yFilter.update(0, estimatedVisionPose.getY());
        headingFilter.update(0, estimatedVisionPose.getRotation().getRadians());

        // Get filtered values
        double filteredX = xFilter.getState();
        double filteredY = yFilter.getState();
        double filteredHeading = headingFilter.getState();

        // Fuse with odometry (weighted average or direct set)
        Pose2d filteredVisionPose = new Pose2d(filteredX, filteredY, new Rotation2d(filteredHeading));
        fusePoseEstimate(filteredVisionPose);
    }

    private void fusePoseEstimate(Pose2d visionPose) {
        Pose2d currentPose = getPose();

        double visionWeight = 0.3; // How much to trust vision vs odometry
        double fusedX = currentPose.getX() * (1 - visionWeight) + visionPose.getX() * visionWeight;
        double fusedY = currentPose.getY() * (1 - visionWeight) + visionPose.getY() * visionWeight;
        double fusedHeading = currentPose.getHeading() * (1 - visionWeight) + visionPose.getHeading() * visionWeight;

        resetPose(new Pose(fusedX, fusedY, fusedHeading));
        // setPoseEstimate(visionPose); Test this if vision is goated or not
    }

    public void resetVisionFilters(double x, double y, double heading) {
        xFilter.reset(x, 0.1, 1.0);
        yFilter.reset(y, 0.1, 1.0);
        headingFilter.reset(heading, 0.1, 1.0);
    }

    /** NOTE THAT THE POSE3D IS IN PEDRO COORDINATES */
    public double getDistanceToPose3D(Pose targetPose, double targetHeight, double turretZ) {
        Pose2d robotPose = getPose();

        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double dz = targetHeight - turretZ;

        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    public static void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory() {
        Drawing.drawPoseHistory(poseHistory);
        drawCurrent();
    }

    public PathBuilder getPathBuilder() {
        return new PathBuilder(follower);
    }

    public double getVelocity() {
        return follower.getVelocity().getMagnitude();
    }

    public Pose2d getPose() {
        return new Pose2d(follower.getPose().getX(), follower.getPose().getY(), Rotation2d.fromRadians(follower.getPose().getHeading()));
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void setMaxPower(final double maxPower) {
        follower.setMaxPower(maxPower);
    }

    public void setMovementVectors(double forward, double strafe, double rotation, boolean isRobotCentric) {
        follower.setTeleOpDrive(forward, strafe, rotation, isRobotCentric);
    }

    public void followTrajectory(final PathChain pathChain, final boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    public void resetDriveSpeed() {
        follower.setTeleOpDrive(0,0,0, false);
    }

    public void resetPose(Pose pose) {
        follower.setPose(pose);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void startTeleopDriving() {
        follower.startTeleopDrive(true);
    }

    public void update() {
        follower.update();
    }

    public void toggleRobotFieldCentric() {
        isRobotCentric = !isRobotCentric;
    }

    public void manualResetPoseForAlliance() {
        resetPose(GlobalConstants.getCurrentAllianceColor() == GlobalConstants.AllianceColor.BLUE ? new Pose(8.75, 7.5, 180).mirror() : new Pose(8.75, 7.5, 180));
    }

    public boolean isFollowingTrajectory() {
        return follower.isBusy();
    }

    public boolean isDrivetrainAtSetpoint() {
        return !follower.isBusy();
    }

}
