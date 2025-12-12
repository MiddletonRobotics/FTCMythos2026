package org.firstinspires.ftc.teamcode.command_factories;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.InstantCommand;
import org.firstinspires.ftc.library.command.RepeatCommand;
import org.firstinspires.ftc.library.command.SequentialCommandGroup;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.subsystems.LED;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;

public class LEDFactory {
    public static Command constantColorCommand(LED led, LEDConstants.ColorValue setpoint) {
        return Commands.runOnce(() -> {
            led.enableSolidColor(setpoint);
        });
    }

    public static Command constantColorCommand(LED led, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            double value = setpoint.getAsDouble();
            led.setPosition(value);
        });
    }
}

