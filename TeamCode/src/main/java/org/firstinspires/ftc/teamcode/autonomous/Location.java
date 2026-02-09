package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public enum Location {
    HUMAN_PLAYER(DrivetrainConstants.kHumanStartingPoseBlue),
    CLOSE(DrivetrainConstants.kCloseGoalStartingPoseBlue),
    FAR(DrivetrainConstants.kFarStartingPoseBlue);

    private final Pose pose;

    private Location(final Pose pose) {
        this.pose = pose;
    }

    public Pose getPose() {
        return pose;
    }
}
