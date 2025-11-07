package org.firstinspires.ftc.library.command.type;

import org.firstinspires.ftc.library.command.CommandBase;

import java.util.function.BooleanSupplier;

public class WaitUntilCommand extends CommandBase {

    private final BooleanSupplier m_condition;

    /**
     * Creates a new WaitUntilCommand that ends after a given condition becomes true.
     *
     * @param condition the condition to determine when to end
     */
    public WaitUntilCommand(BooleanSupplier condition) {
        m_condition = condition;
    }

    @Override
    public boolean isFinished() {
        return m_condition.getAsBoolean();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}