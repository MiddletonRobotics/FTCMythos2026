package org.firstinspires.ftc.library.command.type;

import static org.firstinspires.ftc.library.command.CommandGroupBase.registerGroupedCommands;
import static org.firstinspires.ftc.library.command.CommandGroupBase.requireUngrouped;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.CommandBase;

public class PerpetualCommand extends CommandBase {

    protected final Command m_command;

    /**
     * Creates a new PerpetualCommand.  Will run another command in perpetuity, ignoring that
     * command's end conditions, unless this command itself is interrupted.
     *
     * @param command the command to run perpetually
     */
    public PerpetualCommand(Command command) {
        requireUngrouped(command);
        registerGroupedCommands(command);
        m_command = command;
        m_requirements.addAll(command.getRequirements());
    }

    @Override
    public void initialize() {
        m_command.initialize();
    }

    @Override
    public void execute() {
        m_command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_command.runsWhenDisabled();
    }

}