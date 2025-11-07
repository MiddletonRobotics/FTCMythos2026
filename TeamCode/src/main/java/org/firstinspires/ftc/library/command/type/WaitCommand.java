package org.firstinspires.ftc.library.command.type;

import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.library.command.CommandBase;

import java.util.concurrent.TimeUnit;

public class WaitCommand extends CommandBase {

    protected Timing.Timer m_timer;

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param millis the time to wait, in milliseconds
     */
    public WaitCommand(long millis) {
        m_timer = new Timing.Timer(millis, TimeUnit.MILLISECONDS);
        setName(m_name + ": " + millis + " milliseconds");
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.pause();
    }

    @Override
    public boolean isFinished() {
        return m_timer.done();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}