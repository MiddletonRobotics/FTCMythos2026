package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
    public static final String turretMotorID = "turretMotor";
    public static final double turretGearRatio = (24.0 / 48) * (26.0 / 135);

    public static double P = 0.001;
    public static double I = 0.0;
    public static double D = 0.0;
}
