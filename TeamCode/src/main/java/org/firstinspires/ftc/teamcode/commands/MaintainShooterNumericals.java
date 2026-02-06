package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.library.command.CommandBase;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MaintainShooterNumericals extends CommandBase {
    private final Shooter shooter;
    private final DoubleSupplier flywheelVelocitySetpoint;
    private final DoubleSupplier hoodPositionSetpoint;
    private final BooleanSupplier isFlywheelCommanded;

    public MaintainShooterNumericals(final Shooter shooter, final DoubleSupplier flywheelVelocitySetpoint, final DoubleSupplier hoodPositionSetpoint, final BooleanSupplier isFlywheelCommanded) {
        this.shooter = shooter;
        this.flywheelVelocitySetpoint = flywheelVelocitySetpoint;
        this.hoodPositionSetpoint = hoodPositionSetpoint;
        this.isFlywheelCommanded = isFlywheelCommanded;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if(isFlywheelCommanded.getAsBoolean()) {
            shooter.setVelocitySetpoint(flywheelVelocitySetpoint.getAsDouble());
        } else {
            shooter.setVelocitySetpoint(ShooterConstants.kShooterIdleRPM);
        }

        shooter.setHoodPosition(hoodPositionSetpoint.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
