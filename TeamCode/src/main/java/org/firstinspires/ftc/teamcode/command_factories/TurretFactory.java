package org.firstinspires.ftc.teamcode.command_factories;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TurretFactory {
    public static Command positionSetpointCommand(Turret turret, DoubleSupplier setpoint) {
        return Commands.run(() -> {
            double position = setpoint.getAsDouble();
            turret.setPosition(position);
        }).withName("Turret Position");
    }

    public static Command resetTurretPosition(Turret turret) {
        return Commands.waitUntil(turret::isHomingTriggered).andThen(
            Commands.runOnce(turret::resetPosition)
        );
    }
}
