package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeConstants {
    public static final String kSubsystemName = "Intake ";
    public static final String rearIntakeMotorID = "rearIntakeMotor";
    public static final String frontIntakeMotorID = "frontIntakeMotor";

    public static final double intakeMaximumRPM = 435;
    public static final double kRearIntakeMotorCPR = 384.5;
    public static final double kFrontIntakeMotorCPR = 537.7;

    public static final double frontIntakeRatio = (96 / 24.00);
    public static final double rearIntakeRatio = 1;

    public static double fP = 0.001;
    public static double fI = 0.0;
    public static double fD = 0.0;
    public static double fF = 0.0;

    public static double fS = 0.0;
    public static double fV = 0.0;
    public static double fA = 0.0;

    public static double sP = 0.001;
    public static double sI = 0.0;
    public static double sD = 0.0;
    public static double sF = 0.0;

    public static double sS = 0.0;
    public static double sV = 0.0;
    public static double sA = 0.0;
}
