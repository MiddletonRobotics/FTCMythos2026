package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TransferConstants {
    public static final String kSubsystemName = "Transfer ";

    public static final String blockerServoID = "blockServo";
    public static final String kickerServoID = "kickerServo";
    public static final String firstColorSensorID = "fCS";
    public static final String secondColorSensorID = "sCS";

    public static double blockerEnabled = 0;
    public static double blockerIdlePosition = 0.00;
    public static double kickerIdlePosition = 0.38;

    public static double blockerAllowPosition = 1;
    public static double kickerFeedPosition = 0.0;
}
