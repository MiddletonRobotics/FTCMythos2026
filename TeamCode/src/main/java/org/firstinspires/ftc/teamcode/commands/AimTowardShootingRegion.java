package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.library.command.CommandBase;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AimTowardShootingRegion extends CommandBase {
    private final Turret turret;
    private final Supplier<Pose2d> currentRobotPose;
    private final Supplier<GlobalConstants.AllianceColor> allianceColorSupplier;
    private final BooleanSupplier autoTrackingEnabled;

    public AimTowardShootingRegion(final Turret turret, final Supplier<Pose2d> currentRobotPose, final Supplier<GlobalConstants.AllianceColor> allianceColorSupplier, final BooleanSupplier autoTrackingEnabled) {
        this.turret = turret;
        this.currentRobotPose = currentRobotPose;
        this.allianceColorSupplier = allianceColorSupplier;
        this.autoTrackingEnabled = autoTrackingEnabled;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose targetPose = turret.getTargetPose(allianceColorSupplier.get());

        double desiredAngle = turret.computeAngle(currentRobotPose.get(), targetPose, TurretConstants.kTurretOffsetFromCenterOfRotationX, TurretConstants.kTurretOffsetFromCenterOfRotationY);
        if(autoTrackingEnabled.getAsBoolean()) turret.setPosition(desiredAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
