package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.CommandScheduler;
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

@TeleOp(name="TestTeleop", group = "TeleOp")
public class TestTeleop extends CommandOpMode {

    private Drivetrain drivetrain;

    private Intake intake;

    private Shooter shooter;

    private Transfer transfer;

    private Turret turret;

    private Vision vision;

    private LED led;

    private GamepadEx driverController;

    private GamepadEx operatorController;

    private Telemetry telemetryA;

    @Override
    public void initialize(){
        GlobalConstants.allianceColor = GlobalConstants.AllianceColor.BLUE;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new Drivetrain(hardwareMap, telemetryA);
        intake = new Intake(hardwareMap, telemetryA);
        shooter = new Shooter(hardwareMap, telemetryA);
        transfer = new Transfer(hardwareMap, telemetryA);
        turret = new Turret(hardwareMap, telemetryA);
        vision = new Vision(hardwareMap, telemetryA);
        led = new LED(hardwareMap, telemetryA);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        shooter.onInitialization();
        transfer.onInitialization(true, true);

        drivetrain.setDefaultCommand(new TeleopMecanum(
                drivetrain,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX(),
                () -> true
        ));

        new Trigger(() -> driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 )
                .whenActive(IntakeFactory.openLoopSetpointCommand(intake, () -> 1))
                .whenInactive(IntakeFactory.openLoopSetpointCommand(intake, () -> 0)

        );

        driverController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(IntakeFactory.openLoopSetpointCommand(intake, () -> -1))
                .whenReleased(IntakeFactory.openLoopSetpointCommand(intake, () -> 0)

        );

        new Trigger(() -> driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 )
                .whenActive(ShooterFactory.velocitySetpointCommand(shooter, () -> ShooterConstants.shooterRPM))
                .whenInactive(ShooterFactory.velocitySetpointCommand(shooter, () -> 0)

        );

        driverController.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenActive(
                ShooterFactory.hoodPositionCommand(shooter, () -> ShooterConstants.hoodPosition),
                ShooterFactory.hoodPositionCommand(shooter, () -> 0)

        );

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_UP).toggleWhenActive(
                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerEnabled),
                TransferFactory.engageBlocker(transfer, () -> 0)

        );

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ConstrainedFlashCommand(led, LEDConstants.ColorValue.ORANGE, () -> 500, () -> 100)
        );

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(LEDFactory.constantColorCommand(led, () -> LEDConstants.testColor)
        );

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryA.update();
    }

}
