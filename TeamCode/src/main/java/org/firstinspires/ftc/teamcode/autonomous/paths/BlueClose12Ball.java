package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class BlueClose12Ball {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower, Constants.pathConstraints);
        pathBuilder
                .addPath(new BezierLine(
                        new Pose(25, 127),
                        new Pose(48.15, 98.5)
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(130))
                .addPath(new BezierLine(
                        new Pose(14.46, 83.65),
                        new Pose(56, 50)
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125));

        return pathBuilder.build();
    }
}
