package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.library.command.button.Trigger;
import org.firstinspires.ftc.library.gamepad.GamepadEx;
import org.firstinspires.ftc.library.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.command_factories.IntakeFactory;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.command_factories.TransferFactory;
import org.firstinspires.ftc.teamcode.commands.ConstrainedFlashCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopMecanum;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TeleopBindings {
    private TeleopBindings() {}

    public static void configureBindings(GamepadEx driver, GamepadEx operator, Drivetrain drivetrain, Intake intake, Transfer transfer, Shooter shooter, Turret turret, LED led) {
        /* ------------------------------ Driver Controls ------------------------------ */

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(IntakeFactory.openLoopSetpointCommand(intake, () -> 1))
                .whenInactive(IntakeFactory.openLoopSetpointCommand(intake, () -> 0));

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.75))
                .whenInactive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        new Trigger(() -> transfer.secondCSDistance() < 2.5).whenActive(() -> led.setSolid(LEDConstants.ColorValue.GREEN));
        new Trigger(() -> transfer.firstCSDistance() < 2.5).whenActive(() -> led.setSolid(LEDConstants.ColorValue.YELLOW));

        driver.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenActive(
                () -> shooter.setHoodPosition(0.65),
                () -> shooter.setHoodPosition(ShooterConstants.hoodIdlePosition)
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(IntakeFactory.openLoopSetpointCommand(intake, () -> -1))
                .whenReleased(IntakeFactory.openLoopSetpointCommand(intake, () -> 0));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(() -> shooter.setHoodPosition(shooter.getHoodTargetPosition() + 0.001));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(ShooterFactory.openLoopSetpointCommand(shooter, () -> -0.3))
                .whenReleased(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        /* ------------------------------ Operator Controls ------------------------------ */

        new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 1));

        new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        operator.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(TransferFactory.runKickerCycle(transfer));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(
                () -> transfer.setBlockerPosition(TransferConstants.blockerAllowPosition),
                () -> transfer.setBlockerPosition(TransferConstants.blockerIdlePosition)
        );

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> turret.setManualPower(0.4))
                .whenReleased(() -> turret.setManualPower(0.0));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> turret.setManualPower(-0.4))
                .whenReleased(() -> turret.setManualPower(0.0));

        new  Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.75))
                .whenInactive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));
    }

    public static void configureDefaultCommands(GamepadEx driver, GamepadEx operator, Drivetrain drivetrain, Intake intake, Transfer transfer, Shooter shooter, Turret turret, LED led) {
        drivetrain.setDefaultCommand(new TeleopMecanum(
                drivetrain,
                driver::getLeftY,
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> true
        ));
    }
}
