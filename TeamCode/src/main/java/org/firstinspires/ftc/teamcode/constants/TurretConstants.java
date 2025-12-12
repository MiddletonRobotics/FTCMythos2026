package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    public static final String kSubsystemName = "Turret ";
    public static final String turretMotorID = "turretMotor";
    public static final String homingSwitchID = "tHS";
    public static final double turretGearRatio = (24.0 / 48) * (135.0 / 26.0);

    public static double pP = 0.009;
    public static double pI = 0.0;
    public static double pD = 0.0;
    public static double pF = 0.0;

    public static double sP = 0.001;
    public static double sI = 0.0;
    public static double sD = 0.0011;
    public static double sF = 0.01012;

    public static int pidfSwitch = 10;
    public static double tuningSetpoint = 0;
}
