package org.firstinspires.ftc.library.command.type;

public class PrintCommand extends InstantCommand {
    /**
     * Creates a new a PrintCommand.
     *
     * @param message the message to print
     */
    public PrintCommand(String message) {
        super(() -> System.out.println(message));
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}