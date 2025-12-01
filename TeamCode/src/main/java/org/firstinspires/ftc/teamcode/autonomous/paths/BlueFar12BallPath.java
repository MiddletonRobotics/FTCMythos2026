package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class BlueFar12BallPath {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower, Constants.pathConstraints);
        pathBuilder
                .addPath(new BezierLine(
                        new Pose(56, 8),
                        new Pose(55.6, 96) //path 1
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(130))
                .addPath(new BezierCurve(
                        new Pose(55.6, 96),
                        new Pose(59, 82),
                        new Pose(14.7, 83.9) //path 2
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(14.7, 83.9),
                        new Pose(55.9, 95.6) //path 3
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .addPath(new BezierCurve(
                        new Pose(55.9, 95.6),
                        new Pose(65.6, 57.4),
                        new Pose(14.7, 59) //path 4
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(14.7, 59),
                        new Pose(55.4, 96) //path 5
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .addPath(new BezierCurve(
                        new Pose(55.4, 96),
                        new Pose(71.9,32.5),
                        new Pose(14.7, 35) //path 6
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(14.7, 35),
                        new Pose(54.9, 95.4) //path 7
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .addPath(new BezierLine(
                        new Pose(54.9, 95.4),
                        new Pose(28.34,69.3) // path 8
                )).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(142));


        return pathBuilder.build();
    }
}

