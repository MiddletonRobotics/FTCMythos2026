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
    public static Command setConstantColorCommand(LED led, LEDConstants.ColorValue setpoint) {
        return Commands.runOnce(() -> {
            led.setSolid(setpoint);
        });
    }

    public static Command setStandardBlinkingCommand(LED led, LEDConstants.ColorValue setpoint, LongSupplier intervalMs) {
        return Commands.runOnce(() -> {
            led.setDefaultSimpleBlink(setpoint, intervalMs.getAsLong());
        });
    }

    public static Command setAdvancedStandardBlinkingCommand(LED led, LEDConstants.ColorValue setpointA, LEDConstants.ColorValue setpointB, LongSupplier intervalMs) {
        return Commands.runOnce(() -> {
            led.setSimpleBlink(setpointA, setpointB, intervalMs.getAsLong());
        });
    }
}

