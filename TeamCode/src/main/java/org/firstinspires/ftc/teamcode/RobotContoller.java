package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.button.Trigger;
import org.firstinspires.ftc.library.gamepad.GamepadEx;
import org.firstinspires.ftc.library.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.commands.TeleopMecanum;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp(name="RobotController", group="TeleOp")
public class RobotContoller extends CommandOpMode {
    private Drivetrain drivetrain;
    private DcMotorEx intakeMotor;
    private Transfer transfer;
    private Shooter shooter;

    private GamepadEx driverController;
    private GamepadEx operatorController;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @Override
    public void initialize() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        drivetrain = Drivetrain.getInstance(hardwareMap, telemetryManager);
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.intakeMotorID);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = Transfer.getInstance(hardwareMap, telemetryManager);
        shooter = Shooter.getInstance(hardwareMap, telemetryManager);

        register(drivetrain, transfer, shooter);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        new Trigger(() -> driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(() -> intakeMotor.setPower(1))
                .whenInactive(() -> intakeMotor.setPower(0));

        new Trigger(() -> operatorController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(() -> shooter.setShooterRPM(-4500));

        new Trigger(() -> operatorController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5)
                .whenActive(() -> shooter.setShooterRPM(0));

        operatorController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(() -> transfer.setKickerPosition(TransferConstants.kickerFeedPosition))
            .whenReleased(() -> transfer.setKickerPosition(TransferConstants.kickerIdlePosition));

        operatorController.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenPressed(
                () -> transfer.setBlockerPosition(TransferConstants.blockerAllowPosition),
                () -> transfer.setBlockerPosition(TransferConstants.blockerIdlePosition)
        );

        driverController.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenActive(
                () -> shooter.setHoodPosition(1),
                () -> shooter.setHoodPosition(ShooterConstants.hoodIdlePosition)
        );

        driverController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> intakeMotor.setPower(-1))
                .whenReleased(() -> intakeMotor.setPower(0));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(() -> shooter.setHoodPosition(shooter.getHoodTargetPosition() + 0.001));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(() -> shooter.setShooterRPM(-3000))
            .whenReleased(() -> shooter.setShooterRPM(0));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(() -> shooter.setShooterRPM(-6000))
            .whenReleased(() -> shooter.setShooterRPM(0));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(() -> shooter.setHoodPosition(shooter.getHoodTargetPosition() - 0.001));

        driverController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> shooter.setShooterRPM(1800))
                .whenReleased(() -> shooter.setShooterRPM(0));

        drivetrain.setDefaultCommand(new TeleopMecanum(
                drivetrain,
                () -> driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> true
        ));
    }
}
