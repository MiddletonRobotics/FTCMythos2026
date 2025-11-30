package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class LeaveAutonomousPathing {
    public static PathChain path(Follower follower) {
        PathBuilder pathBuilder = new PathBuilder(follower, Constants.pathConstraints);

        pathBuilder
                .addPath(new BezierLine(
                        new Pose(56, 8),
                        new Pose(68, 5)
                )).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(125))
                .addPath(new BezierLine(
                        new Pose(68, 5),
                        new Pose(56, 50)
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125));

        return pathBuilder.build();
    }
}
