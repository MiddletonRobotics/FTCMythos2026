package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .mass(14.515)
            .forwardZeroPowerAcceleration(-29.859)
            .lateralZeroPowerAcceleration(-55.138)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.225, 0.06896247130821645, 0.0018352227027179874))
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.5,0,0.055,0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.16, 0, 0.02, 0.022))
            .translationalPIDFSwitch(5)
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.1, 0, 0.1, 0.015))
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0.0, 0.00, 0.65, 0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.018, 0.0, 0.000, 0.55, 0.005));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .yVelocity(59.443)
            .xVelocity(81.0930);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.125)
            .strafePodX(-5.314)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 25, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
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

