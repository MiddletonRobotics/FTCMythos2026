package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.gamepad.Gamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.library.utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class RobotContoller extends OpModeCommand {
    private Drivetrain drivetrain;
    private Intake intake;
    private Turret turret;
    private Shooter shooter;

    private Superstructure superstructure;

    private GamepadEx driverController;
    private GamepadEx operatorController;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @Override
    public void initialize() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        drivetrain = Drivetrain.getInstance(hardwareMap, telemetryManager);
        intake = Intake.getInstance(hardwareMap);
        turret = Turret.getInstance(hardwareMap);
        shooter = Shooter.getInstance(hardwareMap);

        superstructure = new Superstructure(drivetrain, intake, turret, shooter, telemetryManager);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        //driverController.getGamepadButton(GamepadKeys.Button.A).whenPressed()

    }
}
