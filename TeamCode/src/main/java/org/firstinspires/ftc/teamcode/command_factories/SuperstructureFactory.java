package org.firstinspires.ftc.teamcode.command_factories;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.ConditionalCommand;
import org.firstinspires.ftc.library.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.concurrent.locks.Condition;
import java.util.function.DoubleSupplier;

public class SuperstructureFactory {
    public static Command shooterSmartVelocityRampCommand(Intake intake, Transfer transfer, Shooter shooter, LED led, DoubleSupplier shooterRPM, DoubleSupplier hoodPosition) {
        return Commands.sequence( // TODO: Add a check to see if we have balls, if not then just return (maybe add it to the blink array)
                new ConditionalCommand(
                        Commands.none(),
                        new ParallelCommandGroup(
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                LEDFactory.setAdvancedStandardBlinkingCommand(led, LEDConstants.ColorValue.ORANGE, LEDConstants.ColorValue.VIOLET, () -> 50)
                        ),
                        transfer::isBlockerEngaged
                ),
                IntakeFactory.openLoopSetpointCommand(intake, () -> 0),
                new ParallelCommandGroup(
                        ShooterFactory.velocitySetpointCommand(shooter, shooterRPM),
                        ShooterFactory.hoodPositionCommand(shooter, hoodPosition),
                        LEDFactory.setStandardBlinkingCommand(led, LEDConstants.ColorValue.YELLOW, () -> 100)
                ),
                LEDFactory.setStandardBlinkingCommand(led, LEDConstants.ColorValue.GREEN, () -> 50)
        );
    }

    public static Command smartIntakingCommand(Intake intake, Transfer transfer, Shooter shooter, LED led, DoubleSupplier shooterRPM, DoubleSupplier hoodPosition) {
        return Commands.sequence(
            IntakeFactory.setFrontIntakeOpenLoopSetpointCommand(intake, () -> 1),
            IntakeFactory.setRearIntakeOpenLoopSetpointCommand(intake, () -> 1),
            Commands.waitUntil(transfer::fir)
        );
    }
}
