package org.firstinspires.ftc.teamcode.autonomous.temp.auto;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.CommandScheduler;
import org.firstinspires.ftc.library.command.InstantCommand;
import org.firstinspires.ftc.library.command.ParallelCommandGroup;
import org.firstinspires.ftc.library.command.RunCommand;
import org.firstinspires.ftc.library.command.SequentialCommandGroup;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.library.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.autonomous.temp.paths.BlueClose12BallPath;
import org.firstinspires.ftc.teamcode.autonomous.temp.paths.RedClose12BallPath;
import org.firstinspires.ftc.teamcode.command_factories.IntakeFactory;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.command_factories.TransferFactory;
import org.firstinspires.ftc.teamcode.command_factories.TurretFactory;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name="BlueClose12Ball", group="auto", preselectTeleOp="RobotController")
public class BlueClose12Ball extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;

    private PathChain currentPathChain;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @Override
    public void initialize() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        drivetrain = new Drivetrain(hardwareMap, telemetryManager);
        intake = new Intake(hardwareMap, telemetryManager);
        transfer = new Transfer(hardwareMap, telemetryManager);
        shooter = new Shooter(hardwareMap, telemetryManager);
        turret = new Turret(hardwareMap, telemetryManager);

        currentPathChain = BlueClose12BallPath.path(drivetrain.follower);

        drivetrain.setStartingPose(new Pose(25.500, 130.500, Math.toRadians(144)));

        transfer.onInitialization(true, false);
        shooter.onInitialization();

        schedule(
                new RunCommand(drivetrain::update),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(0), true, 1),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.825),
                                ShooterFactory.hoodPositionCommand(shooter, () -> ShooterConstants.hoodIdlePosition + 0.2),
                                TurretFactory.positionSetpointCommand(turret, () -> 0),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition)
                        ),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(1), true, 0.8),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.2)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(2), true, 1),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.825),
                                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.2)
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(3), true, 0.8),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.2)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(4), true, 1),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.825),
                                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.2)
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(5), true, 0.8),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.2)
                        )
                )
        );
    }

    @Override
    public void initialize_loop() {
        telemetryManager.update(telemetry);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryManager.update(telemetry);
    }
}
