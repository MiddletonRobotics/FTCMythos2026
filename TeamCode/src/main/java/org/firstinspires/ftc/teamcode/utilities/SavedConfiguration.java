package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.Auto;
import org.firstinspires.ftc.teamcode.autonomous.Location;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;

public class SavedConfiguration {
    private SavedConfiguration() {}

    public static Location selectedLocation = Location.CLOSE;
    public static Auto selectedAuto = Auto.IDLE;
    public static GlobalConstants.AllianceColor selectedAlliance = GlobalConstants.AllianceColor.BLUE;
    public static Pose pathEndPose = new Pose(8, 8, 0);

    public static boolean autoLocked = false;

    public static void clear() {
        autoLocked = false;
    }
}
