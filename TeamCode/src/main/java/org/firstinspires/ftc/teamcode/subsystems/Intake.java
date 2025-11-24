package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private DcMotorEx intakeMotor;

    public static Intake instance;
    public static synchronized Intake getInstance(HardwareMap hMap) {
        if(instance == null) {
            instance = new Intake(hMap);
        }

        return instance;
    }

    private double targetPower;

    private Intake(HardwareMap hMap) {
        intakeMotor = hMap.get(DcMotorEx.class, IntakeConstants.intakeMotorID);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
    }

    public void setIntakeTargetRPM(double power) {
        intakeMotor.setPower(power);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity() / IntakeConstants.intakeMotorCPR;
    }
}
