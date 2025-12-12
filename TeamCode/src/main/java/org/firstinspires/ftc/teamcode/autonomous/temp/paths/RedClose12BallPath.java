package org.firstinspires.ftc.teamcode.autonomous.temp.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class RedClose12BallPath {
    public static PathChain path(Follower follower) {
            PathBuilder pathBuilder = new PathBuilder(follower);
            pathBuilder
                    .addPath(
                            new BezierLine(new Pose(25.500, 130.500).mirror(), new Pose(60.000, 84.000).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(54))
                    .addPath(
                            new BezierLine(new Pose(60.000, 84.000).mirror(), new Pose(12.000, 84.000).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(new Pose(12.000, 84.000).mirror(), new Pose(60.000, 84.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(54))
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 83.000).mirror(),
                                    new Pose(74.000, 56.500).mirror(),
                                    new Pose(15.000, 59.000).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(new Pose(15.000, 60.000), new Pose(60.000, 84.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(54))
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000).mirror(),
                                    new Pose(94.000, 33.250).mirror(),
                                    new Pose(10.000, 36.000).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            return pathBuilder.build();
    }
}