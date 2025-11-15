package org.firstinspires.ftc.library.command.type;

import org.firstinspires.ftc.library.command.CommandBase;
import org.firstinspires.ftc.library.command.Subsystem;

public class StartEndCommand extends CommandBase {

    protected final Runnable m_onInit;
    protected final Runnable m_onEnd;

    /**
     * Creates a new StartEndCommand.  Will run the given runnables when the command starts and when
     * it ends.
     *
     * @param onInit       the Runnable to run on command init
     * @param onEnd        the Runnable to run on command end
     * @param requirements the subsystems required by this command
     */
    public StartEndCommand(Runnable onInit, Runnable onEnd, Subsystem... requirements) {
        m_onInit = onInit;
        m_onEnd = onEnd;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_onInit.run();
    }

    @Override
    public void end(boolean interrupted) {
        m_onEnd.run();
    }

}