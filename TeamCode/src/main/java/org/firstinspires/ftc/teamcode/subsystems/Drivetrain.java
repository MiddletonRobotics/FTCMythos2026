package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.geometry.Pose2d;
import org.firstinspires.ftc.library.geometry.Rotation2d;
import org.firstinspires.ftc.library.geometry.Translation2d;
import org.firstinspires.ftc.library.geometry.Units;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {
    private MotorEx leftFront;
    private MotorEx rightFront;
    private MotorEx leftRear;
    private MotorEx rightRear;

    private IMU imu;
    public Follower follower;

    public enum WantedState {
        TELEOP_DRIVE,
        PEDROPATHING_PATH,
        ROTATION_LOCK,
        DRIVE_TO_POINT,
        ON_THE_FLY,
        IDLE
    }

    private enum SystemState {
        TELEOP_DRIVE,
        PEDROPATHING_PATH,
        ROTATION_LOCK,
        DRIVE_TO_POINT,
        ON_THE_FLY,
        IDLE
    }

    private WantedState wantedState = WantedState.TELEOP_DRIVE;
    private SystemState systemState = SystemState.TELEOP_DRIVE;

    private final PIDFController headingController;
    private final PIDFController teleopDriveToPointController;
    private final PIDFController autonomousDriveToPointController;

    private double desiredHeadingRadians;
    private double maxVelocityOutputForDriveToPoint;
    private Pose2d desiredPoseForDriveToPoint = new Pose2d();

    private double forward = 0.0;
    private double strafe = 0.0;
    private double rotation = 0.0;
    private boolean robotCentric = false;

    private PathChain currentPathFollowing;
    private boolean holdEnd;
    private double maxPower;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private static Drivetrain instance = null;
    public static synchronized Drivetrain getInstance(HardwareMap aHardwareMap, TelemetryManager telemetryManager) {
        if(instance == null) {
            instance = new Drivetrain(aHardwareMap, telemetryManager);
        }

        return instance;
    }

    private Drivetrain(HardwareMap aHardwareMap, TelemetryManager telemetryManager) {
        leftFront = new MotorEx(aHardwareMap, DrivetrainConstants.fLMotorID);
        rightFront = new MotorEx(aHardwareMap, DrivetrainConstants.fRMotorID);
        leftRear = new MotorEx(aHardwareMap, DrivetrainConstants.bLMotorID);
        rightRear = new MotorEx(aHardwareMap, DrivetrainConstants.bRMotorID);

        leftFront.setRunMode(MotorEx.RunMode.RawPower);
        rightFront.setRunMode(MotorEx.RunMode.RawPower);
        leftRear.setRunMode(MotorEx.RunMode.RawPower);
        rightRear.setRunMode(MotorEx.RunMode.RawPower);

        leftFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        leftFront.setInverted(false);
        rightFront.setInverted(false);
        leftRear.setInverted(false);
        rightRear.setInverted(true);

        this.headingController = new PIDFController(0.01, 0, 0, 0);
        this.teleopDriveToPointController = new PIDFController(0.01, 0, 0, 0);
        this.autonomousDriveToPointController = new PIDFController(0.01, 0, 0, 0);

        headingController.setTolerance(Math.toRadians(3));

        follower = Constants.createFollower(aHardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        this.telemetryManager = telemetryManager;

        initializeImu(aHardwareMap);
    }

    public void initializeImu(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);
    }

    @Override
    public void periodic() {
        telemetryManager.debug("Drivetrain Pose X: " + getPose().getX());
        telemetryManager.debug("Drivetrain Pose Y: " + getPose().getY());
        telemetryManager.debug("Drivetrain Pose θ: " + getPose().getRotation().getDegrees());

        systemState = handleStateTransition();
        applyStates();
    }

    private SystemState handleStateTransition() {
         switch(wantedState) {
            case TELEOP_DRIVE:
                if(systemState != SystemState.TELEOP_DRIVE) {
                    resetDriveSpeed();
                    follower.setMaxPower(1);
                    follower.startTeleopDrive(true);
                    return SystemState.TELEOP_DRIVE;
                } else {
                    return SystemState.TELEOP_DRIVE;
                }
            case PEDROPATHING_PATH:
                follower.setMaxPower(maxPower);
                follower.followPath(currentPathFollowing, holdEnd);
                return SystemState.PEDROPATHING_PATH;
            case ROTATION_LOCK:
                return SystemState.ROTATION_LOCK;
            case DRIVE_TO_POINT:
                return SystemState.DRIVE_TO_POINT;
            case ON_THE_FLY:
                resetDriveSpeed();
                return SystemState.ON_THE_FLY;
            default:
                return SystemState.IDLE;
        }
    }

    private void applyStates() {
        switch(systemState) {
            case TELEOP_DRIVE:
                setMovementVectors(forward, strafe, rotation, robotCentric);
                break;
            case PEDROPATHING_PATH:

            case ROTATION_LOCK:
                double currentHeading = follower.getPose().getHeading();
                double maximumRotation = DrivetrainConstants.kMaximumRotationRadiansPerSecond;

                headingController.setSetPoint(desiredHeadingRadians);
                rotation = headingController.calculate(currentHeading);
                rotation = MathUtils.clamp(rotation, -maximumRotation, maximumRotation);

                if(Math.abs(rotation) < 0.02) rotation = 0.0;

                follower.setTeleOpDrive(forward, strafe, rotation, robotCentric);
                follower.update();
            case DRIVE_TO_POINT:
                Translation2d translationToDesiredPoint = desiredPoseForDriveToPoint.getTranslation().minus(new Translation2d(follower.getPose().getX(), follower.getPose().getX()));
                double linearDistance = translationToDesiredPoint.getNorm();
                double frictionConstant = 0.0;

                if(linearDistance <= 0.5) { // If the target translation is 0.5 inches or less
                    frictionConstant = DrivetrainConstants.kDriveToPointStaticFriction * DrivetrainConstants.kMaximumLinearVelocityInchesPerSecond;
                }

                Rotation2d directionOfTravel = new Rotation2d(translationToDesiredPoint.getX(), translationToDesiredPoint.getY());
                double velocityOutput = 0.0;

                if(GlobalConstants.opModeType == GlobalConstants.OpModeType.AUTONOMOUS) {
                    velocityOutput = Math.min(
                            Math.abs(autonomousDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                            maxVelocityOutputForDriveToPoint
                    );
                } else {
                    velocityOutput = Math.min(
                            Math.abs(teleopDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                            maxVelocityOutputForDriveToPoint
                    );
                }

                double xComponent = velocityOutput * directionOfTravel.getCos();
                double yComponent = velocityOutput * directionOfTravel.getSin();

                telemetryManager.addData("Drivetrain/DriveToPoint/xVelocitySetpoint", xComponent);
                telemetryManager.addData("Drivetrain/DriveToPoint/yVelocitySetpoint", yComponent);
                telemetryManager.addData("Drivetrain/DriveToPoint/velocityOutput", velocityOutput);
                telemetryManager.addData("Drivetrain/DriveToPoint/linearDistance", linearDistance);
                telemetryManager.addData("Drivetrain/DriveToPoint/directionOfTravel", directionOfTravel);
                telemetryManager.addData("Drivetrain/DriveToPoint/desiredPoint", desiredPoseForDriveToPoint);

                drive(xComponent, yComponent, desiredPoseForDriveToPoint.getHeading());
            case ON_THE_FLY:
                Path lazyCurveGeneration = new Path(new BezierLine(follower::getPose, new Pose(desiredPoseForDriveToPoint.getX(), desiredPoseForDriveToPoint.getY())));
                lazyCurveGeneration.setHeadingInterpolation(HeadingInterpolator.linear(follower.getHeading(), desiredPoseForDriveToPoint.getHeading()));

                follower.followPath(lazyCurveGeneration);
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setTargetRotation(double targetRotation) {
        desiredHeadingRadians = targetRotation;
        setWantedState(WantedState.ROTATION_LOCK);
    }

    public void setDesiredPoseForDriveToPoint(Pose2d pose) {
        this.desiredPoseForDriveToPoint = pose;
        this.wantedState = WantedState.DRIVE_TO_POINT;
        this.maxVelocityOutputForDriveToPoint = 30; // 30 inches per second
    }

    public void setDesiredPoseForDriveToPoint(Pose2d pose, double maxVelocityOutputForDriveToPoint) {
        this.desiredPoseForDriveToPoint = pose;
        this.wantedState = WantedState.DRIVE_TO_POINT;
        this.maxVelocityOutputForDriveToPoint = 30; // 30 inches per second
    }

    public boolean isAtTargetRotation() {
        return headingController.atSetPoint();
    }

    public boolean isAtTargetRotation(double tolerance) {
        return headingController.getPositionError() < tolerance;
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

        leftFront.set(chassisSpeeds[0]);
        rightFront.set(chassisSpeeds[1]);
        leftRear.set(chassisSpeeds[2]);
        rightRear.set(chassisSpeeds[3]);
    }

    public void setMovementVectors(double forward, double strafe, double rotation, boolean isRobotCentric) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.robotCentric = robotCentric;
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

    public void setStartingPose(Pose2d pose) {
        follower.setStartingPose(pose.getAsPedroPose());
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void followPath(PathChain pathChain, boolean holdEnd, double maxPower) {
        this.currentPathFollowing = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }
}
