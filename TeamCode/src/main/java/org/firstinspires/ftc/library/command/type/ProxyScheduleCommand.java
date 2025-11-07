package org.firstinspires.ftc.library.command.type;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.CommandBase;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class ProxyScheduleCommand extends CommandBase {
    private final Set<Command> m_toSchedule;
    private boolean m_finished;

    /**
     * Creates a new ProxyScheduleCommand that schedules the given commands when initialized,
     * and ends when they are all no longer scheduled.
     *
     * @param toSchedule the commands to schedule
     */
    public ProxyScheduleCommand(Command... toSchedule) {
        m_toSchedule = new HashSet<Command>(Arrays.asList(toSchedule));
    }

    @Override
    public void initialize() {
        for (Command command : m_toSchedule) {
            command.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            for (Command command : m_toSchedule) {
                command.cancel();
            }
        }
    }

    @Override
    public void execute() {
        m_finished = true;
        for (Command command : m_toSchedule) {
            m_finished &= !command.isScheduled();
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}