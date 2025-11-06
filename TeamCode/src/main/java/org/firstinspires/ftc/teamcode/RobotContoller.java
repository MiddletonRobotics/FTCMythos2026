package org.firstinspires.ftc.teamcode;

import com.bylazar.gamepad.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class RobotContoller extends CommandOpMode {
    private GamepadEx driverController;
    private GamepadEx operatorController;

    private GamepadButton intakeButton;
    private GamepadButton stopIntake;
    private GamepadButton startFlywheels;

    private Intake intake = Intake.getInstance(hardwareMap);
    private Shooter shooter = Shooter.getInstance(hardwareMap);

    @Override
    public void initialize() {
        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        intakeButton = new GamepadButton(driverController, GamepadKeys.Button.LEFT_BUMPER);
        stopIntake = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_BUMPER);
        startFlywheels = GamepadButton(driverController, GamepadKeys.Button)


        intakeButton.whenPressed(() -> intake.setIntakeTargetRPM(465));
        stopIntake.whenPressed(() -> intake.setIntakeTargetRPM(0));
    }
}
