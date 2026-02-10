package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.ConditionalCommand;
import org.firstinspires.ftc.library.command.InstantCommand;
import org.firstinspires.ftc.library.command.button.Trigger;
import org.firstinspires.ftc.library.gamepad.GamepadEx;
import org.firstinspires.ftc.library.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.command_factories.IntakeFactory;
import org.firstinspires.ftc.teamcode.command_factories.LiftFactory;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.command_factories.SuperstructureFactory;
import org.firstinspires.ftc.teamcode.command_factories.TransferFactory;
import org.firstinspires.ftc.teamcode.commands.AimTowardShootingRegion;
import org.firstinspires.ftc.teamcode.commands.MaintainShooterNumericals;
import org.firstinspires.ftc.teamcode.commands.TeleopMecanum;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TeleopBindings {
    private TeleopBindings() {}

    public static void configureBindings(GamepadEx driver, GamepadEx operator, Drivetrain drivetrain, Intake intake, Transfer transfer, Shooter shooter, Turret turret, Lift lift, LED led) {
        /* ------------------------------ Driver Controls ------------------------------ */

        if(GlobalConstants.getCurrentDriverType() == GlobalConstants.DriverType.HANISH) {
            new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                    .whenActive(IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1))
                    .whenInactive(IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0));

            new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                    .whenActive(ShooterFactory.velocitySetpointCommand(shooter, () -> 3500))
                    .whenInactive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

            driver.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenActive(
                    () -> shooter.setHoodPosition(-2),
                    () -> shooter.setHoodPosition(2)
            );

            driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                    ShooterFactory.velocitySetpointCommand(shooter, () -> 3350).andThen(SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> true))
            );

            driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> -1))
                    .whenReleased(IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0));

            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenActive(() -> shooter.setHoodPosition(shooter.getHoodTargetPosition() + 0.001));

            driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(ShooterFactory.openLoopSetpointCommand(shooter, () -> -0.3))
                    .whenReleased(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));
        } else {
            new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                    .whenActive(IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1))
                    .whenInactive(IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0));

            new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                    .whenActive(new ConditionalCommand(
                            SuperstructureFactory.rapidFireCommand(intake, transfer, shooter, led, () -> false),
                            SuperstructureFactory.controlledShootCommand(intake, transfer, shooter, led, () -> false),
                            drivetrain::isRobotinCloseZone
                    ));

            driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> -1))
                    .whenReleased(IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0));

            driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).toggleWhenPressed(
                    TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                    TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition)
            );

            driver.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(
                    SuperstructureFactory.auomaticallyParkAndLiftCommand(drivetrain, lift)
            );

            driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                    new InstantCommand(drivetrain::toggleRobotFieldCentric)
            );

            driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                    new InstantCommand(drivetrain::manualResetPoseForAlliance)
            );

            driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                    TransferFactory.runKickerCycle(transfer)
            );

            driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                    () -> drivetrain.resetPoseAndRotate(drivetrain.getPose().getAsPedroPose())
            );
        }

        /* ------------------------------ Operator Controls ------------------------------ */

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(new InstantCommand(() -> drivetrain.setDrivetrainParkingPose(GlobalConstants.getCurrentAllianceColor() == GlobalConstants.AllianceColor.BLUE ? DrivetrainConstants.kTopLeftParkingPoseBlue : DrivetrainConstants.kTopLeftParkingPoseRed)));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive(new InstantCommand(() -> drivetrain.setDrivetrainParkingPose(GlobalConstants.getCurrentAllianceColor() == GlobalConstants.AllianceColor.BLUE ? DrivetrainConstants.kTopRightParkingPoseBlue : DrivetrainConstants.kTopRightParkingPoseRed)));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(new InstantCommand(() -> drivetrain.setDrivetrainParkingPose(GlobalConstants.getCurrentAllianceColor() == GlobalConstants.AllianceColor.BLUE ? DrivetrainConstants.kBottomLeftParkingPoseBlue : DrivetrainConstants.kBottomLeftParkingPoseRed)));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive(new InstantCommand(() -> drivetrain.setDrivetrainParkingPose(GlobalConstants.getCurrentAllianceColor() == GlobalConstants.AllianceColor.BLUE ? DrivetrainConstants.kBottomRightParkingPoseBlue : DrivetrainConstants.kBottomRightParkingPoseRed)));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> lift.setPower(1))
                .whenReleased(() -> lift.setPower(0));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> lift.setPower(-1))
                .whenReleased(() -> lift.setPower(0));

        operator.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(TransferFactory.runKickerCycle(transfer));

        operator.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(LiftFactory.resetLiftToZeroCommand(lift));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).toggleWhenPressed(
                () -> shooter.setFlywheelEnabled(false),
                () -> shooter.setFlywheelEnabled(true)
        );

        //operator.getGamepadButton(GamepadKeys.Button.CIRCLE)
        //        .whenPressed(new InstantCommand(() -> shooter.)

        new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.75))
                .whenInactive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        new Trigger(transfer::doesTransferContainAnyBalls)
                .whenActive(new InstantCommand(() -> shooter.setFlywheelCommanded(true)))
                .whenInactive(new InstantCommand(() -> shooter.setFlywheelCommanded(false)));

        new Trigger(transfer::doesTransferContainAllBalls)
                .whenActive(Commands.sequence(
                        new InstantCommand(() -> driver.gamepad.rumble(1.0, 1.0, 200)),
                        Commands.waitMillis(350),
                        new InstantCommand(() -> driver.gamepad.rumble(1.0, 1.0, 200))
                ));
    }

    public static void configureDefaultCommands(GamepadEx driver, GamepadEx operator, Drivetrain drivetrain, Intake intake, Transfer transfer, Shooter shooter, Turret turret, LED led) {
        if(GlobalConstants.getCurrentOpModeType() == GlobalConstants.OpModeType.TELEOP) {
            drivetrain.setDefaultCommand(new TeleopMecanum(
                    drivetrain,
                    driver::getLeftY,
                    driver::getLeftX,
                    driver::getRightX,
                    drivetrain::isRobotCentric
            ));
        }

        turret.setDefaultCommand(new AimTowardShootingRegion(
                turret,
                drivetrain::getPose,
                GlobalConstants::getCurrentAllianceColor,
                turret::isTurretAutoTrackingEnabled
        ));

        shooter.setDefaultCommand(new MaintainShooterNumericals(
                shooter,
                () -> shooter.calculateFlywheelSpeeds(drivetrain.getDistanceToPose3D(shooter.getTargetPose(GlobalConstants.getCurrentAllianceColor()), 38, 12)),
                () -> shooter.calculateHoodPosition(drivetrain.getDistanceToPose3D(shooter.getTargetPose(GlobalConstants.getCurrentAllianceColor()), 38, 12)),
                shooter::isFlywheelCommanded
        ));
    }
}
