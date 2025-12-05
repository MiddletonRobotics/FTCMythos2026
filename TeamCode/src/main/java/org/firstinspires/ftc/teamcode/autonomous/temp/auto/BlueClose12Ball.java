package org.firstinspires.ftc.teamcode.autonomous.temp.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.CommandScheduler;
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
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@Autonomous(name="BlueClose12Ball", group="auto", preselectTeleOp="RobotController")
public class BlueClose12Ball extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    private PathChain currentPathChain;

    @Override
    public void initialize() {
        drivetrain = Drivetrain.getInstance(hardwareMap, telemetry);
        intake = Intake.getInstance(hardwareMap, telemetry);
        transfer = Transfer.getInstance(hardwareMap, telemetry);
        shooter = Shooter.getInstance(hardwareMap, telemetry);

        currentPathChain = BlueClose12BallPath.path(drivetrain.follower);

        drivetrain.setStartingPose(new Pose(25.500, 130.500, Math.toRadians(144)));

        transfer.onInitialization(true, false);
        shooter.onInitialization();

        register(drivetrain, intake, transfer, shooter);
        schedule(
                new WaitUntilCommand(this::opModeIsActive),
                new RunCommand(drivetrain::update),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(0), true, 1),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.825),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition)
                        ),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(1), true, 1),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.2)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(2), true, 1),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.825),
                                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.4)
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(3), true, 1),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.2)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(4), true, 1),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.825),
                                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.4)
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(5), true, 1),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.2)
                        )
                )
        );
    }

    @Override
    public void initialize_loop() {
        telemetry.addData("Drivetrain Pose X: ", drivetrain.getPose().getX());
        telemetry.addData("Drivetrain Pose Y: ", drivetrain.getPose().getY());
        telemetry.addData("Drivetrain Pose θ: ", drivetrain.getPose().getRotation().getDegrees());
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.update();
    }
}
