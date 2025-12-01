package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class BlueClose12BallPath {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower, Constants.pathConstraints);
        pathBuilder
                .addPath(new BezierLine(
                        new Pose(25, 127),
                        new Pose(48.15, 98.5)
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(130))
                .addPath(new BezierCurve(
                        new Pose(48.15, 98.5),
                        new Pose(63, 81),
                        new Pose(14.46, 83.65)
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(14.46, 83.65),
                        new Pose(48.1, 98.7)
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .addPath(new BezierCurve(
                        new Pose(48.1, 98.7),
                        new Pose(61.7, 54),
                        new Pose(14, 59.4)
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(14, 59.4),
                        new Pose(48, 98.7) //path 5
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .addPath(new BezierCurve(
                        new Pose(48, 98.8),
                        new Pose(61,29.4),
                        new Pose(14.5, 35.5)
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(14.5, 35),
                        new Pose(48.6, 98.5)
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .addPath(new BezierLine(
                        new Pose(48.6, 98.5),
                        new Pose(40.6,63.5)
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(142));


        return pathBuilder.build();
    }
}
