package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.InstantCommand;
import org.firstinspires.ftc.library.command.ParallelCommandGroup;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.library.math.Pair;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.command_factories.IntakeFactory;
import org.firstinspires.ftc.teamcode.command_factories.LEDFactory;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.command_factories.TransferFactory;
import org.firstinspires.ftc.teamcode.commands.ConstrainedFlashCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;

public class AutoFactory {

    private final GlobalConstants.AllianceColor alliance;
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Transfer transfer;
    private final Turret turret;
    private final Shooter shooter;
    private final Vision vision;
    private final LED led;
    private final PathBuilder pathBuilder;

    public AutoFactory(GlobalConstants.AllianceColor alliance, Drivetrain drivetrain, Intake intake, Transfer transfer, Turret turret, Shooter shooter, Vision vision, LED led) {
        this.alliance = alliance;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.transfer = transfer;
        this.turret = turret;
        this.shooter = shooter;
        this.vision = vision;
        this.led = led;

        this.pathBuilder = new PathBuilder(drivetrain.follower);
    }

    Pair<Pose, Command> initializeIdle(Pose startingPose) {
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);
        return Pair.of(refractoredPose, Commands.idle());
    }

    Pair<Pose, Command> initializeCloseLeave(Pose startingPose) {
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);
        PathChain createdPath = pathBuilder
                .addPath(new BezierLine(
                        DrivetrainConstants.decideToFlipPose(alliance, refractoredPose),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue)
                )).setLinearHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, refractoredPose).getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue).getHeading()
                )
                .build();

        return Pair.of(
                refractoredPose,
                Commands.sequence(
                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1)
                )
        );
    }

    Pair<Pose, Command> initializeFarLeave(Pose startingPose) {
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);
        PathChain createdPath = pathBuilder
                .addPath(new BezierLine(
                        DrivetrainConstants.decideToFlipPose(alliance, refractoredPose),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue)
                )).setLinearHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, refractoredPose).getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue).getHeading()
                )
                .build();

        return Pair.of(
                refractoredPose,
                Commands.sequence(
                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1)
                )
        );
    }

    Pair<Pose, Command> initializeFarSixBall(Pose startingPose) {
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);
        drivetrain.setStartingPose(refractoredPose);

        PathChain createdPath = pathBuilder
            .addPath(
                new BezierLine(
                    DrivetrainConstants.decideToFlipPose(alliance, refractoredPose),
                    DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                )
            ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading())
            .addPath(
                new BezierCurve(
                    DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                    DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupControlPositionBlue),
                    DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupPositionBlue)
                )
            ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupPositionBlue).getHeading())
            .addPath(
                new BezierLine(
                    DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupPositionBlue),
                    DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                )
            ).setLinearHeadingInterpolation(
                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupPositionBlue).getHeading(),
                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading()
            )
            .addPath(
                new BezierLine(
                    DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                    DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue)
                )
            ).setLinearHeadingInterpolation(
                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading(),
                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue).getHeading()
            )
            .build();

        return Pair.of(
            refractoredPose,
            Commands.sequence(
                new ParallelCommandGroup(
                    new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1),
                    ShooterFactory.velocitySetpointCommand(shooter, () -> 5600), // TODO: replace with shooter.calculateFlywheelSpeed() later.
                    ShooterFactory.hoodPositionCommand(shooter, () -> 0.36)
                ),
                new WaitCommand(1000),
                IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                new WaitCommand(3000),
                TransferFactory.runKickerCycle(transfer),
                new ParallelCommandGroup(
                    new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 0.8),
                    TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                    ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.2)
                ),
                new ParallelCommandGroup(
                    new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1),
                    ShooterFactory.velocitySetpointCommand(shooter, () -> 5600),
                    IntakeFactory.openLoopSetpointCommand(intake, () -> 1)
                ),
                IntakeFactory.openLoopSetpointCommand(intake, () -> 0),
                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                new WaitCommand(3000),
                TransferFactory.runKickerCycle(transfer),
                new ParallelCommandGroup(
                    new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 0.8),
                    TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                    ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.0),
                    IntakeFactory.openLoopSetpointCommand(intake, () -> 0)
                )
            )
        );
    }

    Pair<Pose, Command> initializeCloseNineBall(Pose startingPose) {
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);
        drivetrain.setStartingPose(refractoredPose);

        PathChain createdPath = pathBuilder
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, refractoredPose),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue).getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading()
                )
                .addPath(
                        new BezierCurve(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoControlPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue).getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading()
                )
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue).getHeading()
                )
                .build();

        return Pair.of(
                refractoredPose,
                Commands.sequence(
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1).alongWith(new InstantCommand(() -> led.enableBlinking(75, LEDConstants.ColorValue.YELLOW))),
                                ShooterFactory.velocitySetpointCommand(shooter, () -> 4400), // TODO: replace with shooter.calculateFlywheelSpeed() later.
                                ShooterFactory.hoodPositionCommand(shooter, () -> 0.48)
                        ),
                        new WaitCommand(500).andThen(LEDFactory.constantColorCommand(led, LEDConstants.ColorValue.INDIGO)),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 0.8).raceWith(new WaitCommand(5000)),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.3)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1).alongWith(new InstantCommand(() -> led.enableBlinking(75, LEDConstants.ColorValue.YELLOW))),
                                ShooterFactory.velocitySetpointCommand(shooter, () -> 4400)
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        LEDFactory.constantColorCommand(led, LEDConstants.ColorValue.INDIGO),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 0.8),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.3)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(4), true, 1).alongWith(new InstantCommand(() -> led.enableBlinking(75, LEDConstants.ColorValue.YELLOW))),
                                ShooterFactory.velocitySetpointCommand(shooter, () -> 4400)
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        LEDFactory.constantColorCommand(led, LEDConstants.ColorValue.INDIGO),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(5), true, 0.8),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.0),
                                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.0)
                        )
                )
        );
    }

    Pair<Pose, Command> initializeCloseNineBallPickup(Pose startingPose) {
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);
        drivetrain.setStartingPose(refractoredPose);

        PathChain createdPath = pathBuilder
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, refractoredPose),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue).getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading()
                )
                .addPath(
                        new BezierCurve(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoControlPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue).getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading()
                )
                .addPath(
                        new BezierCurve(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeControlPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue).getHeading()
                )
                .build();

        return Pair.of(
                refractoredPose,
                Commands.sequence(
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1).alongWith(new InstantCommand(() -> led.enableBlinking(75, LEDConstants.ColorValue.YELLOW))),
                                ShooterFactory.velocitySetpointCommand(shooter, () -> 4400), // TODO: replace with shooter.calculateFlywheelSpeed() later.
                                ShooterFactory.hoodPositionCommand(shooter, () -> 0.48)
                        ),
                        new WaitCommand(500).andThen(LEDFactory.constantColorCommand(led, LEDConstants.ColorValue.INDIGO)),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 0.8).raceWith(new WaitCommand(5000)),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.3)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1).alongWith(new InstantCommand(() -> led.enableBlinking(75, LEDConstants.ColorValue.YELLOW))),
                                ShooterFactory.velocitySetpointCommand(shooter, () -> 4400)
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        LEDFactory.constantColorCommand(led, LEDConstants.ColorValue.INDIGO),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 0.8),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.3)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(4), true, 1).alongWith(new InstantCommand(() -> led.enableBlinking(75, LEDConstants.ColorValue.YELLOW))),
                                ShooterFactory.velocitySetpointCommand(shooter, () -> 4400)
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        LEDFactory.constantColorCommand(led, LEDConstants.ColorValue.INDIGO),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(5), true, 0.8),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.0)
                        )
                )
        );
    }
}