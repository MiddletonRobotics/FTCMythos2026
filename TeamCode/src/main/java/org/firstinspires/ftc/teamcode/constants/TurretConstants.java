package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    public static final String kSubsystemName = "Turret ";
    public static final String turretMotorID = "turretMotor";
    public static final String homingSwitchID = "tHS";
    public static final double turretGearRatio = (48.0 / 24) * (135.0 / 26.0);

    public static double P = 0.001;
    public static double I = 0.0;
    public static double D = 0.0;
}
