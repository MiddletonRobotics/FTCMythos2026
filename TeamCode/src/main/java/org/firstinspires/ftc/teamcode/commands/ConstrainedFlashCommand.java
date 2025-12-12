package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.command_factories.LEDFactory.constantColorCommand;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.CommandBase;
import org.firstinspires.ftc.library.command.RepeatCommand;
import org.firstinspires.ftc.library.command.SequentialCommandGroup;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.subsystems.LED;

import java.util.function.IntSupplier;
import java.util.function.LongSupplier;

/**
 * Command that constructs a RepeatCommand at runtime using suppliers for time/count,
 * then delegates execution to that child command. Evaluates suppliers only on initialize().
 */

public class ConstrainedFlashCommand extends CommandBase {
    private final LED led;
    private final LEDConstants.ColorValue setpoint;
    private final LongSupplier milliseconds;
    private final IntSupplier maxCount;

    private Command child;

    public ConstrainedFlashCommand(LED led, LEDConstants.ColorValue setpoint, LongSupplier milliseconds, IntSupplier maxCount) {
        this.led = led;
        this.setpoint = setpoint;
        this.milliseconds = milliseconds;
        this.maxCount = maxCount;

        addRequirements(led);
    }

    @Override
    public void initialize() {
        long ms = Math.max(0, milliseconds.getAsLong());
        int count = Math.max(0, maxCount.getAsInt());

        Command flash = new SequentialCommandGroup(
                constantColorCommand(led, setpoint),
                new WaitCommand(ms),
                constantColorCommand(led, LEDConstants.ColorValue.OFF),
                new WaitCommand(ms)
        );

        child = new RepeatCommand(flash, count);
        child.schedule();
    }

    @Override
    public boolean isFinished() {
        return child == null || child.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (child != null) child.end(interrupted);
    }
}