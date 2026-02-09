package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LiftConstants {
    public static final String kSubsystemName = "Lift ";

    public static final String kLiftServoID = "liftServo";
    public static final String kLiftServoEncoderID = "lSE";
    public static final String kLiftHomingSwitchID = "lHS";

    public static double kP = 1.00;
    public static double kI = 0.00; // This system has insane torque it might actually be needed
    public static double kD = 0.00;
    public static double kF = 0.00;

    public static double kS = 0.00; // This system has insane torque it might actually be needed
    public static double kV = 0.00;
    public static double kA = 0.00;

    public static final double liftServoUpPosition = 0.0;
    public static final double liftServoDownPosition = 36.0;


}