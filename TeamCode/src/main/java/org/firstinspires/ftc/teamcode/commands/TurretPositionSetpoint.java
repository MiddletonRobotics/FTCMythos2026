package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.library.command.CommandBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretPositionSetpoint extends CommandBase {
    private final Turret turret;
    private final PIDFController primaryController;
    private final PIDFController secondaryController;

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Pose2d> targetSupplier;

    public TurretPositionSetpoint(final Turret turret, final Supplier<Pose2d> poseSupplier, final Supplier<Pose2d> targetSupplier) {
        this.turret = turret;
        this.poseSupplier = poseSupplier;
        this.targetSupplier = targetSupplier;

        primaryController = new PIDFController(TurretConstants.pP, TurretConstants.pI, TurretConstants.pD, TurretConstants.pF);
        secondaryController = new PIDFController(TurretConstants.sP, TurretConstants.sI, TurretConstants.sD, TurretConstants.sF);

        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Pose2d position = poseSupplier.get();
        Pose2d target = targetSupplier.get();
        turret.setPosition(turret.computeAngle(position, target, 0, 0));
    }
}