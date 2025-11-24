package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.library.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopMecanum extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier robotCentric;

    public TeleopMecanum(Drivetrain drivetrain, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier robotCentric) {
        this.drivetrain = drivetrain;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentric = robotCentric;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.startTeleopDriving();
    }

    @Override
    public void execute() {
        drivetrain.setMovementVectors(
                forwardSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                rotationSupplier.getAsDouble(),
                robotCentric.getAsBoolean()
        );

        drivetrain.follower.update();
    }
}
