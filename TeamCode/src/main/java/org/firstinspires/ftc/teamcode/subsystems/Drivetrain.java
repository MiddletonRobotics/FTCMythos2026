package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.library.math.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class Drivetrain extends SubsystemBase {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;

    private IMU imu;
    public Follower follower;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    public Drivetrain(HardwareMap hMap, TelemetryManager telemetryManager) {
        leftFront = hMap.get(DcMotorEx.class, DrivetrainConstants.fLMotorID);
        rightFront = hMap.get(DcMotorEx.class, DrivetrainConstants.fRMotorID);
        leftRear = hMap.get(DcMotorEx.class, DrivetrainConstants.bLMotorID);
        rightRear = hMap.get(DcMotorEx.class, DrivetrainConstants.bRMotorID);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        follower = Constants.createFollower(hMap);
        this.telemetryManager = telemetryManager;

        initializeImu(hMap);
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
        telemetryManager.addData(DrivetrainConstants.kSubsystemName + "Pose X", getPose().getX());
        telemetryManager.addData(DrivetrainConstants.kSubsystemName + "Pose Y: ", getPose().getY());
        telemetryManager.addData(DrivetrainConstants.kSubsystemName + "Pose θ: ", getPose().getRotation().getDegrees());
    }

    public void drive(double xSpeedInchesPerSecond, double ySpeedInchesPerSecond, double omegaSpeedRadiansPerSecond) {
        double scaledForward = xSpeedInchesPerSecond / DrivetrainConstants.kMaximumLinearVelocityInchesPerSecond;
        double scaledStrafe = ySpeedInchesPerSecond / DrivetrainConstants.kMaximumLinearVelocityInchesPerSecond;
        double scaledRotation = omegaSpeedRadiansPerSecond / DrivetrainConstants.kMaximumRotationRadiansPerSecond;

        double[] chassisSpeeds = new double[] {
                (scaledForward + scaledRotation + scaledStrafe),
                (scaledForward - scaledRotation - scaledStrafe),
                (scaledForward + scaledRotation - scaledStrafe),
                (scaledForward - scaledRotation + scaledStrafe)
        };

        leftFront.setPower(chassisSpeeds[0]);
        rightFront.setPower(chassisSpeeds[1]);
        leftRear.setPower(chassisSpeeds[2]);
        rightRear.setPower(chassisSpeeds[3]);
    }

    public void startTeleopDriving() {
        follower.startTeleopDrive(true);
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
        drive(0,0,0);
    }

    public void resetPose(Pose2d pose) {
        follower.setPose(pose.getAsPedroPose());
    }

    public Pose2d getPose() {
        return new Pose2d(follower.getPose().getX(), follower.getPose().getY(), new Rotation2d(follower.getPose().getHeading()));
    }

    public void setPose(Pose2d pose) {
        follower.setPose(pose.getAsPedroPose());
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void update() {
        follower.update();
    }

    public boolean isFollowingTrajectory() {
        return follower.isBusy();
    }
}
