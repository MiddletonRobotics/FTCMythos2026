package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.library.math.geometry.Rotation2d;

@Configurable
public class TurretConstants {
    public static final String kSubsystemName = "Turret ";
    public static final String turretMotorID = "turretMotor";
    public static final String homingSwitchID = "tHS";
    public static final double turretGearRatio = (24.0 / 48) * (135.0 / 26.0);

    public static final Pose aimPoseBlue = new Pose(0, 144);
    public static final Pose aimPoseRed = new Pose(144, 144);

    public static double pP = 0.35;
    public static double pI = 0.02;
    public static double pD = 0.0;
    public static double pF = 0.0;

    public static double sP = 0.08;
    public static double sI = 0.002;
    public static double sD = 0.03;
    public static double sF = 0.0;

    public static double pidfSwitch = Math.PI / 18;
    public static double tuningSetpoint = 0;
}
