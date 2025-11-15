package org.firstinspires.ftc.library.command.type;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.CommandBase;
import org.firstinspires.ftc.library.command.Subsystem;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

public class DeferredCommand extends CommandBase {
    private final Supplier<Command> supplier;
    @Nullable
    private Command command;

    /**
     * Creates a new DeferredCommand that directly runs the supplied command when initialized, and
     * ends when it ends. Useful for lazily creating commands when the DeferredCommand is initialized,
     * such as if the supplied command depends on runtime state. The {@link Supplier} will be called
     * each time this command is initialized. The Supplier <i>must</i> create a new Command each call.
     *
     * @param supplier     The command supplier
     * @param requirements The command requirements. This is a {@link List} to prevent accidental
     *                     omission of command requirements. Use {@link Arrays#asList} to easily construct requirements.
     *                     Passing null or an empty list will result in no requirements.
     */
    public DeferredCommand(@NonNull Supplier<Command> supplier, @Nullable List<Subsystem> requirements) {
        this.supplier = Objects.requireNonNull(supplier);
        // Using List argument instead of Set for Java 8 compat.
        if (requirements != null)
            m_requirements.addAll(requirements);
    }

    @Override
    public void initialize() {
        Command cmd = supplier.get();
        if (cmd != null) {
            command = cmd;
        } else {
            throw new NullPointerException("DeferredCommand: Supplied command was null!");
        }
        command.initialize();
    }

    @Override
    public void execute() {
        if (command != null)
            command.execute();
    }

    @Override
    public boolean isFinished() {
        return command == null || command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (command != null)
            command.end(interrupted);
        command = null;
    }
}