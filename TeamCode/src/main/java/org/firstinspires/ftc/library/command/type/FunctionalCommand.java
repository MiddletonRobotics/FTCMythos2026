package org.firstinspires.ftc.library.command.type;

import org.firstinspires.ftc.library.command.CommandBase;
import org.firstinspires.ftc.library.command.Subsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class FunctionalCommand extends CommandBase {

    protected final Runnable m_onInit;
    protected final Runnable m_onExecute;
    protected final Consumer<Boolean> m_onEnd;
    protected final BooleanSupplier m_isFinished;

    /**
     * Creates a new FunctionalCommand.
     *
     * @param onInit       the function to run on command initialization
     * @param onExecute    the function to run on command execution
     * @param onEnd        the function to run on command end
     * @param isFinished   the function that determines whether the command has finished
     * @param requirements the subsystems required by this command
     */
    public FunctionalCommand(Runnable onInit, Runnable onExecute, Consumer<Boolean> onEnd,
                             BooleanSupplier isFinished, Subsystem... requirements) {
        m_onInit = onInit;
        m_onExecute = onExecute;
        m_onEnd = onEnd;
        m_isFinished = isFinished;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_onInit.run();
    }

    @Override
    public void execute() {
        m_onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        m_onEnd.accept(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_isFinished.getAsBoolean();
    }

}