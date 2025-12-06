package org.firstinspires.ftc.teamcode.command_factories;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.subsystems.LED;

import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

public class LEDFactory {
    public static Command timedFlashCommand(LED led, DoubleSupplier setpoint, LongSupplier milliseconds) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    double color = setpoint.getAsDouble();
                    led.setColor(color);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(LEDConstants.offValue);
                }),
                new WaitCommand(milliseconds.getAsLong())
        );
    }
}
