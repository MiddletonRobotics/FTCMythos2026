package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TransferConstants {
    public static final String kSubsystemName = "Transfer ";

    public static final String blockerServoID = "blockServo";
    public static final String kickerServoID = "kickerServo";
    public static final String firstColorSensorID = "fCS";
    public static final String secondColorSensorID = "sCS";

    public static double blockerEnabled = 0;
    public static double blockerIdlePosition = 0.05;
    public static double kickerIdlePosition = 0.5;

    public static double blockerAllowPosition = 0.24;
    public static double kickerFeedPosition = 0.0;
}
