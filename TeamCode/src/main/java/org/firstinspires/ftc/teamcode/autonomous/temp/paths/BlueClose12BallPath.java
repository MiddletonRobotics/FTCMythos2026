package org.firstinspires.ftc.teamcode.autonomous.temp.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class BlueClose12BallPath {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower);
        pathBuilder
                .addPath(
                        new BezierLine(new Pose(25.500, 130.500), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(12.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(new Pose(12.000, 84.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(74.000, 53.500),
                                new Pose(15.000, 55.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(new Pose(15.000, 60.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(94.000, 30.250),
                                new Pose(10.000, 28.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(new Pose(10.000, 30.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        return pathBuilder.build();
    }
}