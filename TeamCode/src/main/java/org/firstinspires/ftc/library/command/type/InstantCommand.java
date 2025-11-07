package org.firstinspires.ftc.library.command.type;

import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.library.command.CommandBase;

public class InstantCommand extends CommandBase {

    private final Runnable m_toRun;

    /**
     * Creates a new InstantCommand that runs the given Runnable with the given requirements.
     *
     * @param toRun        the Runnable to run
     * @param requirements the subsystems required by this command
     */
    public InstantCommand(Runnable toRun, Subsystem... requirements) {
        m_toRun = toRun;

        addRequirements(requirements);
    }

    /**
     * Creates a new InstantCommand with a Runnable that does nothing.  Useful only as a no-arg
     * constructor to call implicitly from subclass constructors.
     */
    public InstantCommand() {
        m_toRun = () -> {
        };
    }

    @Override
    public void initialize() {
        m_toRun.run();
    }

    @Override
    public final boolean isFinished() {
        return true;
    }

}