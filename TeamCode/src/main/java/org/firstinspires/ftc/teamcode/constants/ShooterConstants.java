package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    public static final String shooterMotorID = "shooterMotor";
    public static final String hoodServoID = "hoodServo";
    public static final String blockerServoID = "blockServo";

    public static final double shooterReadyRPM = 600;
    public static final double hoodIdlePosition = 0.4;

    public static final double shooterMotorMaximumRPM = 6000;
    public static final double shooterMotorCPR = 28;

    public static double P = 0.001;
    public static double I = 0.0;
    public static double D = 0.0;
}
