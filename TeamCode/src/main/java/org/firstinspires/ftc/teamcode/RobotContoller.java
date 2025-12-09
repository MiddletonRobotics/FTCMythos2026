package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.CommandScheduler;
import org.firstinspires.ftc.library.command.button.Trigger;
import org.firstinspires.ftc.library.gamepad.GamepadEx;
import org.firstinspires.ftc.library.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.command_factories.IntakeFactory;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.command_factories.TransferFactory;
import org.firstinspires.ftc.teamcode.commands.TeleopMecanum;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="RobotController", group="TeleOp")
public class RobotContoller extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;

    private GamepadEx driverController;
    private GamepadEx operatorController;

    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

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

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(() -> shooter.setHoodPosition(shooter.getHoodTargetPosition() - 0.001));

        driverController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(ShooterFactory.openLoopSetpointCommand(shooter, () -> -0.3))
                .whenReleased(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

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
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
