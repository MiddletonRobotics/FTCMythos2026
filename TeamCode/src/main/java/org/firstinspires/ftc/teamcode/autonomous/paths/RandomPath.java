package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class RandomPath {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower);

        return pathBuilder
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(56.000, 8.000), new Pose(76.142, 76.619))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(76.142, 76.619), new Pose(50.177, 102.425))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(50.177, 102.425), new Pose(108.478, 102.903))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}
