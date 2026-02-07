package org.firstinspires.ftc.teamcode.command_factories;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftFactory {
    public static Command resetLiftToZeroCommand(Lift lift) {
        return Commands.startEnd(lift::resetToZero, lift::stopHoming, lift).until(lift::isHomingSwitchTriggered);
    }
}
