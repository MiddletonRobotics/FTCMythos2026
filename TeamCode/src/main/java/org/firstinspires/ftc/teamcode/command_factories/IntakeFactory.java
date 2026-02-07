package org.firstinspires.ftc.teamcode.command_factories;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class IntakeFactory {
    public static Command setUnevenOpenLoopSetpointCommand(Intake intake, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            intake.setOpenLoopSetpoint(setpoint.getAsDouble());
        }).withName("Intake Open Loop");
    }

    public static Command setEvenOpenLoopSetpointCommand(Intake intake, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            intake.setEqualOpenLoopSetpoint(setpoint.getAsDouble());
        }).withName("Intake Open Loop");
    }

    public static Command setFrontIntakeOpenLoopSetpointCommand(Intake intake, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            intake.setFrontMotorOpenLoop(setpoint.getAsDouble());
        }).withName("Intake Open Loop");
    }

    public static Command setRearIntakeOpenLoopSetpointCommand(Intake intake, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            intake.setRearMotorOpenLoop(setpoint.getAsDouble());
        }).withName("Intake Open Loop");
    }

    public static Command setFrontIntakeVelocitySetpointCommand(Intake intake, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            intake.setFrontVelocitySetpoint(setpoint.getAsDouble());
        }).withName("Intake Open Loop");
    }

    public static Command setRearIntakeVelocitySetpointCommand(Intake intake, DoubleSupplier setpoint) {
        return Commands.runOnce(() -> {
            intake.setRearVelocitySetpoint(setpoint.getAsDouble());
        }).withName("Intake Open Loop");
    }
}
