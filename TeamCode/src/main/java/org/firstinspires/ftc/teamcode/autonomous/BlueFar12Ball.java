package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autonomous.paths.BlueClose12BallPath;
import org.firstinspires.ftc.teamcode.autonomous.paths.BlueFar12BallPath;
import org.firstinspires.ftc.teamcode.autonomous.paths.LeaveAutonomousPathing;
import org.firstinspires.ftc.teamcode.autonomous.paths.TwelveBallAutonomousPathing;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import kotlin.time.Instant;

@Autonomous(name="BlueThreeBallClose", group="SimpleAutonomous")
public class BlueFar12Ball extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    private PathChain currentPathChain;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @Override
    public void initialize() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        drivetrain = Drivetrain.getInstance(hardwareMap, telemetryManager);
        intake = Intake.getInstance(hardwareMap);
        transfer = Transfer.getInstance(hardwareMap, telemetryManager);
        shooter = Shooter.getInstance(hardwareMap, telemetryManager);

        drivetrain.follower.setStartingPose(new Pose(22, 124, Math.toRadians(144)));
        currentPathChain = BlueFar12BallPath.path(drivetrain.follower);

        register(drivetrain, intake, transfer, shooter);
        schedule(
                new RunCommand(drivetrain.follower::update),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(0), true, 1),
                                        new InstantCommand(() -> shooter.setShooterRPM(-4500))
                                ),
                                new WaitCommand(500),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(-1)),
                                new WaitCommand(500),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(0)).andThen(new WaitCommand(1000)),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(-1)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(0)).andThen(new WaitCommand(1000)),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(-1)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> transfer.setKickerPosition(TransferConstants.kickerFeedPosition)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> transfer.setKickerPosition(TransferConstants.kickerIdlePosition)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(0)),
                                new WaitCommand(500),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(-1)),
                                new WaitCommand(500),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(0)).andThen(new WaitCommand(1000)),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(-1)),
                                new WaitCommand(500),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(0)).andThen(new WaitCommand(1000)),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(-1)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> transfer.setKickerPosition(TransferConstants.kickerFeedPosition)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> transfer.setKickerPosition(TransferConstants.kickerIdlePosition)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> intake.setIntakeTargetRPM(0)),
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(1), true, 1),
                                new InstantCommand(() -> shooter.setShooterRPM(0))


                        )
                )
        );
    }
}