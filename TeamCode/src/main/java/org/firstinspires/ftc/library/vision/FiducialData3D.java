package org.firstinspires.ftc.library.vision;

import com.pedropathing.geometry.Pose;

public class FiducialData3D {
    public final Pose robotPose;
    public final int fiducialId;

    public FiducialData3D(Pose robotPose, int fiducialId) {
        this.fiducialId = fiducialId;
        this.robotPose = robotPose;
    }

    public Pose getRobotPose() {
        return robotPose;
    }
}
