package org.firstinspires.ftc.teamcode.command_factories;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.subsystems.LED;

import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

public class LEDFactory {
    public static Command timedFlashCommand(LED led, LEDConstants.ColorValue setpoint, LongSupplier milliseconds) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    led.setColor(setpoint);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(LEDConstants.ColorValue.OFF);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(setpoint);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(LEDConstants.ColorValue.OFF);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(setpoint);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(LEDConstants.ColorValue.OFF);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(setpoint);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(LEDConstants.ColorValue.OFF);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(setpoint);
                }),
                new WaitCommand(milliseconds.getAsLong()),
                Commands.runOnce(() -> {
                    led.setColor(LEDConstants.ColorValue.OFF);
                }),
                new WaitCommand(milliseconds.getAsLong())
        );
    }

    public static Command constantColorCommand(LED led, LEDConstants.ColorValue setpoint) {
        return Commands.runOnce(() -> {
            led.setColor(setpoint);
        });
    }
}
