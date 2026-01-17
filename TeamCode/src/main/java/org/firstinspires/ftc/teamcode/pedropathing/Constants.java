package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13)
            .forwardZeroPowerAcceleration(-32.56245288633697)
            .lateralZeroPowerAcceleration(-90.18623185448504)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.5,0,0.055,0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.015, 0.017))
            .translationalPIDFSwitch(4)
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.1, 0, 0.1, 0.015))
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0.0, 0.00, 0.6, 0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0.0, 0.002 , 0.6, 0.009));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .yVelocity(60.020)
            .xVelocity(80.649);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.0029496848331306146)
            .strafeTicksToInches(0.00294119983011017)
            .turnTicksToInches(0.002816489315462149)
            .leftPodY(6.125)
            .rightPodY(-6.125)
            .strafePodX(-5.314)
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("rightRear")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 25, 1.7, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
    /*
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.7)
            .forwardZeroPowerAcceleration(-42.072)
            .lateralZeroPowerAcceleration(-67.58)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.17,0,0.006,0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.004, 0.015))
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.05, 0.01))
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0.0, 0.01, 0.6, 0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.08, 0.0, 0.002, 0.6, 0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(DrivetrainConstants.fRMotorID)
            .rightRearMotorName(DrivetrainConstants.bRMotorID)
            .leftRearMotorName(DrivetrainConstants.bLMotorID)
            .leftFrontMotorName(DrivetrainConstants.fLMotorID)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.259)
            .strafePodX(-5.314)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
     */
}

