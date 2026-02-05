package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
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
    public static boolean kTuningMode = true;

    public static final Pose kBlueGoalPose = new Pose(9, 138);
    public static final Pose kRedGoalPose = kBlueGoalPose.mirror();
}
