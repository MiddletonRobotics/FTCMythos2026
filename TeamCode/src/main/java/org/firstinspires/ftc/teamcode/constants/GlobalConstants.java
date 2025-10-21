package org.firstinspires.ftc.teamcode.constants;

public class GlobalConstants {
    public enum OpModeType {
        AUTONOMOUS,
        TELEOP
    }
    public enum AllianceColor {
        RED,
        BLUE
    }

    public static OpModeType opModeType;
    public static AllianceColor allianceColor = AllianceColor.BLUE;
}
