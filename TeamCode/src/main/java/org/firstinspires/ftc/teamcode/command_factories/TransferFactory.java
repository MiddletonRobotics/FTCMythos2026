package org.firstinspires.ftc.teamcode.command_factories;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.InstantCommand;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.function.DoubleSupplier;

public class TransferFactory {
    public static Command runKickerCycle(Transfer transfer) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    transfer.setKickerPosition(TransferConstants.kickerFeedPosition);
                }),
                new WaitCommand(1000),
                new InstantCommand(() -> transfer.setKickerPosition(TransferConstants.kickerIdlePosition)),
                new WaitCommand(1000)
        );
    }

    public static Command engageBlocker(Transfer transfer, DoubleSupplier setpoint) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    double position = setpoint.getAsDouble();
                    transfer.setBlockerPosition(position);
                }),
                new WaitCommand(1000)
        );
    }
}
