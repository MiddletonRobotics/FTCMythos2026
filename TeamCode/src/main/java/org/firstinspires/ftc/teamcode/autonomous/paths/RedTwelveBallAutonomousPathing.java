package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class RedTwelveBallAutonomousPathing {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower, Constants.pathConstraints);

        pathBuilder
                .addPath(new BezierLine(
                        new Pose(22.000, 124.000).mirror(),
                        new Pose(50.000, 92.000).mirror()
                )).setConstantHeadingInterpolation(Math.toRadians(36))
                .addPath(new BezierCurve(
                        new Pose(50, 92.000).mirror(),
                        new Pose(79.25, 80.000).mirror(),
                        new Pose(18.5, 83.000).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0));

        return pathBuilder.build();
    }
}
