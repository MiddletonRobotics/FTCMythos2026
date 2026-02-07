package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TransferConstants {
    public static final String kSubsystemName = "Transfer ";
    public static final String blockerServoID = "blockServo";
    public static final String kickerServoID = "kickerServo";

    public static final String firstColorSensorID = "fCS";
    public static final String firstBeamBreakID = "sBB";
    public static final String secondBeamBreakID = "tBB";

    public static final double blockerEnabled = 0;
    public static final double blockerIdlePosition = 0.95;
    public static final double kickerIdlePosition = 0.56;

    public static final double blockerAllowPosition = 0;
    public static final double kickerFeedPosition = 0.1;

    public static final double kFirstColorSensorDistanceThreshold = 1.8; // In inches
    public static final double kSensorDebounceTime = 0.6;
}
