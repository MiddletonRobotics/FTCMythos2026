package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.CommandScheduler;
import org.firstinspires.ftc.library.command.RepeatCommand;
import org.firstinspires.ftc.library.command.RunCommand;
import org.firstinspires.ftc.library.command.button.Trigger;
import org.firstinspires.ftc.library.gamepad.GamepadEx;
import org.firstinspires.ftc.library.gamepad.GamepadKeys;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command_factories.IntakeFactory;
import org.firstinspires.ftc.teamcode.command_factories.LEDFactory;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.command_factories.TransferFactory;
import org.firstinspires.ftc.teamcode.commands.ConstrainedFlashCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopMecanum;
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

@TeleOp(name="RobotController", group="TeleOp")
public class RobotContoller extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private LED led;

    private GamepadEx driverController;
    private GamepadEx operatorController;

    private Telemetry telemetryA;

    @Override
    public void initialize() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new Drivetrain(hardwareMap, telemetryA);
        intake = new Intake(hardwareMap, telemetryA);
        transfer = new Transfer(hardwareMap, telemetryA);
        shooter = new Shooter(hardwareMap, telemetryA);
        turret = new Turret(hardwareMap, telemetryA);
        vision = new Vision(hardwareMap, telemetryA);
        led = new LED(hardwareMap, telemetryA);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        shooter.onInitialization();
        transfer.onInitialization(true, true);

        new Trigger(() -> driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(IntakeFactory.openLoopSetpointCommand(intake, () -> 1))
                .whenInactive(IntakeFactory.openLoopSetpointCommand(intake, () -> 0));

        driverController.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenActive(
                () -> shooter.setHoodPosition(0.65),
                () -> shooter.setHoodPosition(ShooterConstants.hoodIdlePosition)
        );

        driverController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(IntakeFactory.openLoopSetpointCommand(intake, () -> -1))
                .whenReleased(IntakeFactory.openLoopSetpointCommand(intake, () -> 0));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(() -> shooter.setHoodPosition(shooter.getHoodTargetPosition() + 0.001));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.5))
            .whenReleased(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(ShooterFactory.openLoopSetpointCommand(shooter, () -> 1))
            .whenReleased(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        driverController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(ShooterFactory.openLoopSetpointCommand(shooter, () -> -0.3))
                .whenReleased(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new ConstrainedFlashCommand(led, LEDConstants.ColorValue.ORANGE, () -> 125, () -> 20));

        new Trigger(() -> operatorController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.75));

        new Trigger(() -> operatorController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        operatorController.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(TransferFactory.runKickerCycle(transfer));

        operatorController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(
                () -> transfer.setBlockerPosition(TransferConstants.blockerAllowPosition),
                () -> transfer.setBlockerPosition(TransferConstants.blockerIdlePosition)
        );

        operatorController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(() -> turret.setManualPower(-0.4))
            .whenReleased(() -> turret.setManualPower(0.0));

        operatorController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> turret.setManualPower(0.4))
                .whenReleased(() -> turret.setManualPower(0.0));

        drivetrain.setDefaultCommand(new TeleopMecanum(
                drivetrain,
                () -> driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> true
        ));

        schedule(new RunCommand(telemetryA::update));
    }

    @Override
    public void initialize_loop() {
        if(driverController.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            GlobalConstants.allianceColor = GlobalConstants.AllianceColor.RED;
        } else if(driverController.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            GlobalConstants.allianceColor = GlobalConstants.AllianceColor.RED;
        }

        // All subsystem onInititalizationLoop runs here (specifically the LED)
    }
}
