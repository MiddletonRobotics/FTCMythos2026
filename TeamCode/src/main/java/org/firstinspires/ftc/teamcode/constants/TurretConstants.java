package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.library.math.geometry.Rotation2d;

@Configurable
public class TurretConstants {
    public static final String kSubsystemName = "Turret ";
    public static final String turretMotorID = "turretMotor";
    public static final String homingSwitchID = "tHS";
    public static final double turretGearRatio = (24.0 / 48.0) * (135.0 / 26.0);

    public static double pP = 2.5;
    public static double pI = 0.0;
    public static double pD = 0.1;
    public static double pF = 0.093;

    public static double tuningSetpoint = 0;

    public static final double kTurretOffsetFromCenterOfRotationX = -3;
    public static final double kTurretOffsetFromCenterOfRotationY = 0.00;
    public static final double kTurretOffsetFromFloorZ = 14.00;
}
