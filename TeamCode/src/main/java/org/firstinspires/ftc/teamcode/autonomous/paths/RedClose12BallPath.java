package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class RedClose12BallPath {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower, Constants.pathConstraints);
        pathBuilder
                .addPath(new BezierLine(
                        new Pose(25, 127).mirror(),
                        new Pose(48.15, 98.5).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(50))
                .addPath(new BezierCurve(
                        new Pose(48.15, 98.5).mirror(),
                        new Pose(63, 81).mirror(),
                        new Pose(14.46, 83.65).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(14.46, 83.65).mirror(),
                        new Pose(48.1, 98.7).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .addPath(new BezierCurve(
                        new Pose(48.1, 98.7).mirror(),
                        new Pose(61.7, 54).mirror(),
                        new Pose(14, 59.4).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(14, 59.4).mirror(),
                        new Pose(48, 98.7).mirror() //path 5
                )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .addPath(new BezierCurve(
                        new Pose(48, 98.8).mirror(),
                        new Pose(61,29.4).mirror(),
                        new Pose(14.5, 35.5).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(14.5, 35).mirror(),
                        new Pose(48.6, 98.5).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .addPath(new BezierLine(
                        new Pose(48.6, 98.5).mirror(),
                        new Pose(40.6,63.5).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(40));


        return pathBuilder.build();
    }
}
