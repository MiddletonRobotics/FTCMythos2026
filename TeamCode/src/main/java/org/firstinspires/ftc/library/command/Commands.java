package org.firstinspires.ftc.library.command;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static java.util.Objects.requireNonNull;

import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.PrintCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.library.command.type.ConditionalCommand;
import org.firstinspires.ftc.library.command.type.InstantCommand;
import org.firstinspires.ftc.library.command.type.ParallelCommandGroup;
import org.firstinspires.ftc.library.command.type.ParallelDeadlineGroup;
import org.firstinspires.ftc.library.command.type.ParallelRaceGroup;
import org.firstinspires.ftc.library.command.type.SequentialCommandGroup;
import org.firstinspires.ftc.library.command.type.WaitCommand;
import org.firstinspires.ftc.library.command.type.WaitUntilCommand;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Namespace for command factory methods.
 *
 * <p>For convenience, you might want to static import the members of this class.
 */
public final class Commands {
    /**
     * Constructs a command that does nothing, finishing immediately.
     *
     * @return the command
     */
    public static Command none() {
        return new InstantCommand();
    }

    /**
     * Constructs a command that does nothing until interrupted.
     *
     * @param requirements Subsystems to require
     * @return the command
     */
    public static com.seattlesolvers.solverslib.command.Command idle(Subsystem... requirements) {
        return run(() -> {}, requirements);
    }

    // Action Commands

    /**
     * Constructs a command that runs an action once and finishes.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @return the command
     * @see InstantCommand
     */
    public static Command runOnce(Runnable action, Subsystem... requirements) {
        return new InstantCommand(action, requirements);
    }

    /**
     * Constructs a command that runs an action every iteration until interrupted.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @return the command
     * @see RunCommand
     */
    public static com.seattlesolvers.solverslib.command.Command run(Runnable action, Subsystem... requirements) {
        return new RunCommand(action, requirements);
    }

    /**
     * Constructs a command that runs an action once and another action when the command is
     * interrupted.
     *
     * @param start the action to run on start
     * @param end the action to run on interrupt
     * @param requirements subsystems the action requires
     * @return the command
     * @see StartEndCommand
     */
    public static com.seattlesolvers.solverslib.command.Command startEnd(Runnable start, Runnable end, Subsystem... requirements) {
        return new StartEndCommand(start, end, requirements);
    }

    /**
     * Constructs a command that runs an action every iteration until interrupted, and then runs a
     * second action.
     *
     * @param run the action to run every iteration
     * @param end the action to run on interrupt
     * @param requirements subsystems the action requires
     * @return the command
     */
    public static com.seattlesolvers.solverslib.command.Command runEnd(Runnable run, Runnable end, Subsystem... requirements) {
        requireNonNullParam(end, "end", "Command.runEnd");
        return new FunctionalCommand(() -> {}, run, interrupted -> end.run(), () -> false, requirements);
    }

    /**
     * Constructs a command that runs an action once, and then runs an action every iteration until
     * interrupted.
     *
     * @param start the action to run on start
     * @param run the action to run every iteration
     * @param requirements subsystems the action requires
     * @return the command
     */
    public static com.seattlesolvers.solverslib.command.Command startRun(Runnable start, Runnable run, Subsystem... requirements) {
        return new FunctionalCommand(start, run, interrupted -> {}, () -> false, requirements);
    }

    /**
     * Constructs a command that prints a message and finishes.
     *
     * @param message the message to print
     * @return the command
     * @see PrintCommand
     */
    public static com.seattlesolvers.solverslib.command.Command print(String message) {
        return new PrintCommand(message);
    }

    // Idling Commands

    /**
     * Constructs a command that does nothing, finishing after a specified duration.
     *
     * @param seconds after how long the command finishes
     * @return the command
     * @see WaitCommand
     */
    public static Command waitSeconds(long seconds) {
        return new WaitCommand(seconds);
    }

    /**
     * Constructs a command that does nothing, finishing after a specified duration.
     *
     * @param time after how long the command finishes
     * @return the command
     * @see WaitCommand
     */
    public static Command waitTime(long time) {
        return new WaitCommand(time);
    }

    /**
     * Constructs a command that does nothing, finishing once a condition becomes true.
     *
     * @param condition the condition
     * @return the command
     * @see WaitUntilCommand
     */
    public static Command waitUntil(BooleanSupplier condition) {
        return new WaitUntilCommand(condition);
    }

    // Selector Commands

    /**
     * Runs one of two commands, based on the boolean selector function.
     *
     * @param onTrue the command to run if the selector function returns true
     * @param onFalse the command to run if the selector function returns false
     * @param selector the selector function
     * @return the command
     * @see ConditionalCommand
     */
    public static Command either(Command onTrue, Command onFalse, BooleanSupplier selector) {
        return new ConditionalCommand(onTrue, onFalse, selector);
    }

    /**
     * Runs the command supplied by the supplier.
     *
     * @param supplier the command supplier
     * @param requirements the set of requirements for this command
     * @return the command
     * @see DeferredCommand
     */
    public static com.seattlesolvers.solverslib.command.Command defer(Supplier<com.seattlesolvers.solverslib.command.Command> supplier, List<Subsystem> requirements) {
        return new DeferredCommand(supplier, requirements);
    }

    /**
     * Constructs a command that schedules the command returned from the supplier when initialized,
     * and ends when it is no longer scheduled. The supplier is called when the command is
     * initialized.
     *
     * @param supplier the command supplier
     * @return the command
     * @see DeferredCommand
     */
    public static com.seattlesolvers.solverslib.command.Command deferredProxy(Supplier<com.seattlesolvers.solverslib.command.Command> supplier) {
        return defer(() -> supplier.get().asProxy(), List.of());
    }

    // Command Groups

    /**
     * Runs a group of commands in series, one after the other.
     *
     * @param commands the commands to include
     * @return the command group
     * @see SequentialCommandGroup
     */
    public static Command sequence(Command... commands) {
        return new SequentialCommandGroup(commands);
    }

    /**
     * Runs a group of commands at the same time. Ends once all commands in the group finish.
     *
     * @param commands the commands to include
     * @return the command
     * @see ParallelCommandGroup
     */
    public static Command parallel(Command... commands) {
        return new ParallelCommandGroup(commands);
    }

    /**
     * Runs a group of commands at the same time. Ends once any command in the group finishes, and
     * cancels the others.
     *
     * @param commands the commands to include
     * @return the command group
     * @see ParallelRaceGroup
     */
    public static Command race(Command... commands) {
        return new ParallelRaceGroup(commands);
    }

    /**
     * Runs a group of commands at the same time. Ends once a specific command finishes, and cancels
     * the others.
     *
     * @param deadline the deadline command
     * @param otherCommands the other commands to include
     * @return the command group
     * @see ParallelDeadlineGroup
     * @throws IllegalArgumentException if the deadline command is also in the otherCommands argument
     */
    public static Command deadline(Command deadline, Command... otherCommands) {
        return new ParallelDeadlineGroup(deadline, otherCommands);
    }

    private Commands() {
        throw new UnsupportedOperationException("This is a utility class");
    }

    public static <T> T requireNonNullParam(T obj, String paramName, String methodName) {
        return requireNonNull(
                obj,
                "Parameter "
                        + paramName
                        + " in method "
                        + methodName
                        + " was null when it"
                        + " should not have been!  Check the stacktrace to find the responsible line of code - "
                        + "usually, it is the first line of user-written code indicated in the stacktrace.  "
                        + "Make sure all objects passed to the method in question were properly initialized -"
                        + " note that this may not be obvious if it is being called under "
                        + "dynamically-changing conditions!  Please do not seek additional technical assistance"
                        + " without doing this first!");
    }
}