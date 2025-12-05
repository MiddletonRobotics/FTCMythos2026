package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.InstantCommand;
import org.firstinspires.ftc.library.math.Pair;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.ArrayList;

public class AutoFactory {

    private final GlobalConstants.AllianceColor alliance;
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Transfer transfer;
    private final Shooter shooter;
    private final PathBuilder pathBuilder;

    public AutoFactory(GlobalConstants.AllianceColor alliance, Drivetrain drivetrain, Intake intake, Transfer transfer, Shooter shooter) {
        this.alliance = alliance;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.transfer = transfer;
        this.shooter = shooter;
        this.pathBuilder = new PathBuilder(drivetrain.follower);
    }

    Pair<Pose, Command> initializeIdle(Pose startingPose) {
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);
        return Pair.of(refractoredPose, Commands.idle());
    }

    Pair<Pose, Command> initializeThreeBall(Pose startingPose) {
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);
        PathChain createdPath = pathBuilder
                .addPath(new BezierLine(
                        DrivetrainConstants.decideToFlipPose(alliance, refractoredPose),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                )).setConstantHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading()
                )
                .build();

        return Pair.of(
                refractoredPose,
                Commands.sequence(
                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1),
                        ShooterFactory.velocitySetpointCommand(shooter, () -> 1)
                )
        );
    }
}