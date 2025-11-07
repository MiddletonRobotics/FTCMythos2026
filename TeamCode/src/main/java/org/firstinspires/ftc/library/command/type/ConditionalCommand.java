package org.firstinspires.ftc.library.command.type;

import static org.firstinspires.ftc.library.command.CommandGroupBase.requireUngrouped;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.CommandBase;
import org.firstinspires.ftc.library.command.CommandGroupBase;

import java.util.function.BooleanSupplier;

public class ConditionalCommand extends CommandBase {

    private final Command m_onTrue;
    private final Command m_onFalse;
    private final BooleanSupplier m_condition;
    private Command m_selectedCommand;

    /**
     * Creates a new ConditionalCommand.
     *
     * @param onTrue    the command to run if the condition is true
     * @param onFalse   the command to run if the condition is false
     * @param condition the condition to determine which command to run
     */
    public ConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        requireUngrouped(onTrue, onFalse);

        CommandGroupBase.registerGroupedCommands(onTrue, onFalse);

        m_onTrue = onTrue;
        m_onFalse = onFalse;
        m_condition = condition;
        m_requirements.addAll(m_onTrue.getRequirements());
        m_requirements.addAll(m_onFalse.getRequirements());
    }

    @Override
    public void initialize() {
        if (m_condition.getAsBoolean()) {
            m_selectedCommand = m_onTrue;
        } else {
            m_selectedCommand = m_onFalse;
        }
        m_selectedCommand.initialize();
    }

    @Override
    public void execute() {
        m_selectedCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_selectedCommand.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_onTrue.runsWhenDisabled() && m_onFalse.runsWhenDisabled();
    }

}
