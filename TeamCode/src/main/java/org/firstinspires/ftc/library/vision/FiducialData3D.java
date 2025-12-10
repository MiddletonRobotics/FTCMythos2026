package org.firstinspires.ftc.library.vision;

import com.pedropathing.geometry.Pose;

public class FiducialData3D {
    public final Pose robotPose;
    public final double distanceMeters;
    public final int fiducialId;

    public FiducialData3D(Pose robotPose, double distanceMeters, int fiducialId) {
        this.fiducialId = fiducialId;
        this.robotPose = robotPose;
        this.distanceMeters = distanceMeters;
    }
}
