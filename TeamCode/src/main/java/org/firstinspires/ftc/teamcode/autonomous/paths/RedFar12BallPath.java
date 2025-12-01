package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class RedFar12BallPath {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower, Constants.pathConstraints);
        pathBuilder
                .addPath(new BezierLine(
                        new Pose(56, 8).mirror(),
                        new Pose(55.6, 96).mirror() //path 1
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(50))
                .addPath(new BezierCurve(
                        new Pose(55.6, 96).mirror(),
                        new Pose(59, 82).mirror(),
                        new Pose(14.7, 83.9).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(14.7, 83.9).mirror(),
                        new Pose(55.9, 95.6).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .addPath(new BezierCurve(
                        new Pose(55.9, 95.6).mirror(),
                        new Pose(65.6, 57.4).mirror(),
                        new Pose(14.7, 59).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(14.7, 59).mirror(),
                        new Pose(55.4, 96).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .addPath(new BezierCurve(
                        new Pose(55.4, 96).mirror(),
                        new Pose(71.9,32.5).mirror(),
                        new Pose(14.7, 35).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(14.7, 35).mirror(),
                        new Pose(54.9, 95.4).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .addPath(new BezierLine(
                        new Pose(54.9, 95.4).mirror(),
                        new Pose(28.34,69.3).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(40));


        return pathBuilder.build();
    }
}

