package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class TwelveBallAutonomousPathing {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower, Constants.pathConstraints);

        pathBuilder
                .addPath(new BezierLine(
                        new Pose(22.000, 124.000),
                        new Pose(57, 87)
                )).setConstantHeadingInterpolation(Math.toRadians(144))
                .addPath(new BezierCurve(
                        new Pose(57, 87),
                        new Pose(85, 78.000),
                        new Pose(18.5, 81.000)
                )).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180));

        return pathBuilder.build();
    }
}