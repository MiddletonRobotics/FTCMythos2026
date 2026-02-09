package org.firstinspires.ftc.teamcode.autonomous;

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
import org.firstinspires.ftc.teamcode.command_factories.IntakeFactory;
import org.firstinspires.ftc.teamcode.command_factories.LEDFactory;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.command_factories.SuperstructureFactory;
import org.firstinspires.ftc.teamcode.command_factories.TransferFactory;
import org.firstinspires.ftc.teamcode.command_factories.TurretFactory;
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

public class AutoFactory {
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Transfer transfer;
    private final Turret turret;
    private final Shooter shooter;
    private final Vision vision;
    private final LED led;

    public AutoFactory(Drivetrain drivetrain, Intake intake, Transfer transfer, Turret turret, Shooter shooter, Vision vision, LED led) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.transfer = transfer;
        this.turret = turret;
        this.shooter = shooter;
        this.vision = vision;
        this.led = led;
    }

    public Pair<Pose, Pair<Pose, Command>> initializeIdle(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);
        return Pair.of(refractoredPose, Pair.of(refractoredPose, Commands.idle()));
    }

    public Pair<Pose, Pair<Pose, Command>> initializeCloseLeave(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        PathBuilder pathBuilder = drivetrain.getPathBuilder();
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);

        PathChain createdPath = pathBuilder
                .addPath(new BezierLine(
                        refractoredPose,
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue)
                )).setLinearHeadingInterpolation(
                        refractoredPose.getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue).getHeading()
                )
                .build();

        return Pair.of(
                refractoredPose,
                Pair.of(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue),
                        Commands.sequence(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1)
                        )
                )
        );
    }

    public Pair<Pose, Pair<Pose, Command>> initializeFarLeave(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        PathBuilder pathBuilder = drivetrain.getPathBuilder();
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);

        PathChain createdPath = pathBuilder
                .addPath(new BezierLine(
                        refractoredPose,
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue)
                )).setLinearHeadingInterpolation(
                        refractoredPose.getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue).getHeading()
                )
                .build();

        return Pair.of(
                refractoredPose,
                Pair.of(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue),
                        Commands.sequence(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1)
                        )
                )
        );
    }

    public Pair<Pose, Pair<Pose, Command>> initializeFarSixBall(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        PathBuilder pathBuilder = drivetrain.getPathBuilder();
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);

        PathChain createdPath = pathBuilder
                .addPath(
                        new BezierLine(
                                refractoredPose,
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        refractoredPose.getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading()
                )
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOneReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOneReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOneReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOnePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOnePositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOnePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue).getHeading())
                .build();

        return Pair.of(
                refractoredPose,
                Pair.of(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue),
                        Commands.sequence(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1),
                                SuperstructureFactory.controlledShootCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 1),
                                SuperstructureFactory.controlledShootCommand(intake, transfer, shooter, led, () -> false),
                                new InstantCommand(turret::disableTurretAutoTracking),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0),
                                new ParallelCommandGroup(
                                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(4), true, 1),
                                        TurretFactory.positionSetpointCommand(turret, () -> 0.0)
                                )
                        )
                )
        );
    }

    public Pair<Pose, Pair<Pose, Command>> initializeFarNineBall(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        PathBuilder pathBuilder = drivetrain.getPathBuilder();
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);

        PathChain createdPath = pathBuilder
                .addPath(
                        new BezierLine(
                                refractoredPose,
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        refractoredPose.getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading()
                )
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOneReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOneReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOneReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOnePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOnePositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupOnePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue).getHeading())
                .build();

        return Pair.of(
                refractoredPose,
                Pair.of(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue),
                        Commands.sequence(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1),
                                SuperstructureFactory.controlledShootCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 1),
                                SuperstructureFactory.controlledShootCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(4), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(5), true, 1).withTimeout(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(6), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(7), true, 1).withTimeout(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(8), true, 1),
                                SuperstructureFactory.controlledShootCommand(intake, transfer, shooter, led, () -> false),
                                new InstantCommand(turret::disableTurretAutoTracking),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0),
                                new ParallelCommandGroup(
                                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(9), true, 1),
                                        TurretFactory.positionSetpointCommand(turret, () -> 0.0)
                                )
                        )
                )
        );
    }

    public Pair<Pose, Pair<Pose, Command>> initializeFarNineBallMasquerade(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        PathBuilder pathBuilder = drivetrain.getPathBuilder();
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);

        PathChain createdPath = pathBuilder
                .addPath(
                        new BezierLine(
                                refractoredPose,
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        refractoredPose.getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading()
                )
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarReadyThreePositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarReadyThreePositionBlue).getHeading()
                )
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarReadyThreePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupThreePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupThreePositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarPickupThreePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue).getHeading())
                .build();

        return Pair.of(
                refractoredPose,
                Pair.of(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoFarParkingPositionBlue),
                        Commands.sequence(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1),
                                SuperstructureFactory.controlledShootCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1).withTimeout(500),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(4), true, 1).withTimeout(500),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(5), true, 1),
                                SuperstructureFactory.controlledShootCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(6), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(7), true, 1).withTimeout(1800),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(8), true, 1),
                                SuperstructureFactory.controlledShootCommand(intake, transfer, shooter, led, () -> false),
                                new InstantCommand(turret::disableTurretAutoTracking),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0),
                                new ParallelCommandGroup(
                                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(9), true, 1),
                                        TurretFactory.positionSetpointCommand(turret, () -> 0.0)
                                )
                        )
                )
        );
    }

    public Pair<Pose, Pair<Pose, Command>> initializeCloseNineBall(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        PathBuilder pathBuilder = drivetrain.getPathBuilder();
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);

        PathChain createdPath = pathBuilder
                .addPath(
                        new BezierLine(
                                refractoredPose,
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        refractoredPose.getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading()
                )
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
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue).getHeading())
                .build();

        return Pair.of(
                refractoredPose,
                Pair.of(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue),
                        Commands.sequence(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(4), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(5), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                new InstantCommand(turret::disableTurretAutoTracking),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0),
                                new ParallelCommandGroup(
                                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(6), true, 1),
                                        TurretFactory.positionSetpointCommand(turret, () -> 0.0)
                                )
                        )
                )
        );
    }

    public Pair<Pose, Pair<Pose, Command>> initializeCloseNineBallPickup(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        PathBuilder pathBuilder = drivetrain.getPathBuilder();
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);

        PathChain createdPath = pathBuilder
                .addPath(
                        new BezierLine(
                                refractoredPose,
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        refractoredPose.getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading()
                )
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
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue).getHeading())
                .build();

        return Pair.of(
                refractoredPose,
                Pair.of(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue),
                        Commands.sequence(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(4), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(5), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                new InstantCommand(turret::disableTurretAutoTracking),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1.0),
                                new ParallelCommandGroup(
                                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(6), true, 1),
                                        TurretFactory.positionSetpointCommand(turret, () -> 0.0)
                                ),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(7), true, 1),
                                new WaitCommand(1000),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0.0)
                        )
                )
        );
    }

    public Pair<Pose, Pair<Pose, Command>> initializeCloseTwelveBall(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        PathBuilder pathBuilder = drivetrain.getPathBuilder();
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);

        PathChain createdPath = pathBuilder
                .addPath(
                        new BezierLine(
                                refractoredPose,
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        refractoredPose.getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading()
                )
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePrepareGateBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePrepareGateBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePrepareGateBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseGateBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseGateBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseGateBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue).getHeading())
                .build();

        return Pair.of(
                refractoredPose,
                Pair.of(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue),
                        Commands.sequence(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(4), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(5), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(6), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(7), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(8), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(9), true, 1),
                                new WaitCommand(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(10), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                new InstantCommand(turret::disableTurretAutoTracking),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0),
                                new ParallelCommandGroup(
                                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(11), true, 1),
                                        TurretFactory.positionSetpointCommand(turret, () -> 0.0)
                                )
                        )
                )
        );
    }

    public Pair<Pose, Pair<Pose, Command>> initializeCloseTwelveDuckTapeBall(GlobalConstants.AllianceColor alliance, Pose startingPose) {
        PathBuilder pathBuilder = drivetrain.getPathBuilder();
        Pose refractoredPose = DrivetrainConstants.decideToFlipPose(alliance, startingPose);

        PathChain createdPath = pathBuilder
                .addPath(
                        new BezierLine(
                                refractoredPose,
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setLinearHeadingInterpolation(
                        refractoredPose.getHeading(),
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading()
                )
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupOnePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePrepareGateBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePrepareGateBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePrepareGateBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseGateBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseGateBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseGateBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePrepareGateBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePrepareGateBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePrepareGateBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseGateBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseGateBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseGateBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupTwoPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeReadyPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeReadyPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreeReadyPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoClosePickupThreePositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue).getHeading())
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseShootingPositionBlue),
                                DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue)
                        )
                ).setConstantHeadingInterpolation(DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue).getHeading())
                .build();

        return Pair.of(
                refractoredPose,
                Pair.of(
                        DrivetrainConstants.decideToFlipPose(alliance, DrivetrainConstants.kAutoCloseParkingPositionBlue),
                        Commands.sequence(
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(0), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(1), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(2), true, 1).withTimeout(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(3), true, 1).withTimeout(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(4), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(5), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(6), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(8), true, 1).withTimeout(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(9), true, 1).withTimeout(1000),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(10), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(11), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(12), true, 1),
                                new FollowTrajectoryCommand(drivetrain, createdPath.getPath(13), true, 1),
                                SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                                new InstantCommand(turret::disableTurretAutoTracking),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0),
                                new ParallelCommandGroup(
                                        new FollowTrajectoryCommand(drivetrain, createdPath.getPath(14), true, 1),
                                        TurretFactory.positionSetpointCommand(turret, () -> 0.0)
                                )
                        )
                )
        );
    }
}