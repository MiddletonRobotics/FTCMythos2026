package org.firstinspires.ftc.teamcode.autonomous.temp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.lights.LightsManager;
import com.bylazar.lights.PanelsLights;
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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.temp.paths.BlueClose12BallPath;
import org.firstinspires.ftc.teamcode.command_factories.IntakeFactory;
import org.firstinspires.ftc.teamcode.command_factories.LEDFactory;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.command_factories.TransferFactory;
import org.firstinspires.ftc.teamcode.command_factories.TurretFactory;
import org.firstinspires.ftc.teamcode.commands.ConstrainedFlashCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Autonomous(name="BlueClose12Ball", group="auto", preselectTeleOp="RobotController")
public class BlueClose12Ball extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private LED led;
    private Vision vision;

    private PathChain currentPathChain;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @IgnoreConfigurable
    static LightsManager lightsManager;

    @Override
    public void initialize() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        lightsManager = PanelsLights.INSTANCE.getLights();
        GlobalConstants.allianceColor = GlobalConstants.AllianceColor.BLUE;

        drivetrain = new Drivetrain(hardwareMap, telemetryManager);
        intake = new Intake(hardwareMap, telemetryManager);
        transfer = new Transfer(hardwareMap, telemetryManager);
        shooter = new Shooter(hardwareMap, telemetryManager);
        turret = new Turret(hardwareMap, telemetryManager);
        led = new LED(hardwareMap, telemetryManager, lightsManager);
        vision = new Vision(hardwareMap, telemetryManager);

        currentPathChain = BlueClose12BallPath.path(drivetrain.follower);

        drivetrain.setStartingPose(new Pose(25.500, 130.500, Math.toRadians(144)));

        transfer.onInitialization(true, false);
        shooter.onInitialization();
        led.setSolid(GlobalConstants.allianceColor == GlobalConstants.AllianceColor.BLUE ? LEDConstants.ColorValue.BLUE : LEDConstants.ColorValue.RED);

        schedule(
                new RunCommand(drivetrain::update),
                new RunCommand(led::update),
                new RunCommand(() -> turret.setManualPower(0.09)),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(() -> led.setDefaultSimpleBlink(LEDConstants.ColorValue.YELLOW, 100)),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(0), true, 1),
                                ShooterFactory.velocitySetpointCommand(shooter, () -> 4100).andThen(new InstantCommand(() -> led.setSolid(LEDConstants.ColorValue.GREEN))),
                                ShooterFactory.hoodPositionCommand(shooter, () -> 0.44),
                                new InstantCommand(() -> turret.setManualPower(0.09))
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        new WaitCommand(500).andThen(LEDFactory.constantColorCommand(led, LEDConstants.ColorValue.GREEN)),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(1), true, 0.8).raceWith(new WaitCommand(5000)),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.2)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(2), true, 0.8),
                                ShooterFactory.velocitySetpointCommand(shooter, () -> 4100),
                                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.8),
                                new InstantCommand(() -> turret.setManualPower(0.09))
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                        LEDFactory.constantColorCommand(led, LEDConstants.ColorValue.GREEN),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(3), true, 0.8).raceWith(new WaitCommand(5000)),
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.2)
                        ),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drivetrain, currentPathChain.getPath(4), true, 1).raceWith(new ConstrainedFlashCommand(led, LEDConstants.ColorValue.YELLOW, () -> 100, () -> 20)),
                                ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.85),
                                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.8),
                                new InstantCommand(() -> turret.setManualPower(0.09))
                        ),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                        LEDFactory.constantColorCommand(led, LEDConstants.ColorValue.GREEN),
                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                        IntakeFactory.openLoopSetpointCommand(intake, () -> 1),
                        new WaitCommand(3000),
                        TransferFactory.runKickerCycle(transfer)
                )
        );
    }

    public void initialize_loop() {
        if(turret.isHomingTriggered()) {
            turret.resetPosition();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryManager.update(telemetry);
    }
}
