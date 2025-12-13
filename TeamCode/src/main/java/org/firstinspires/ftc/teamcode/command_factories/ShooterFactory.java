package org.firstinspires.ftc.teamcode.command_factories;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class ShooterFactory {
    public static Command velocitySetpointCommand(Shooter shooter, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            double velocity = setpoint.getAsDouble();
            shooter.setVelocitySetpoint(velocity);
        }).andThen(new WaitCommand(2000)).withName("Shooter Velocity");
    }

    public static Command openLoopSetpointCommand(Shooter shooter, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            shooter.setOpenLoopSetpoint(setpoint.getAsDouble());
        }).withName("Shooter Open Loop");
    }

    public static Command hoodPositionCommand(Shooter shooter, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            shooter.setHoodPosition(setpoint.getAsDouble());
        }).withName("Shooter Hood Position");
    }
}
