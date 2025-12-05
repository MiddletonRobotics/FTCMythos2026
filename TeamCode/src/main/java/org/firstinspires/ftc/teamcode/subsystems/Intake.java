package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private DcMotorEx intakeMotor;

    private Telemetry telemetry;

    public static Intake instance;
    public static Intake getInstance(HardwareMap hMap, Telemetry telemetry) {
        if(instance == null) {
            instance = new Intake(hMap, telemetry);
        }

        return instance;
    }

    private double targetPower;

    private Intake(HardwareMap hMap, Telemetry telemetry) {
        intakeMotor = hMap.get(DcMotorEx.class, IntakeConstants.intakeMotorID);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
    }

    public void setVelocitySetpoint(double targetRPM) {
        telemetry.addData("ShooterVelocitySetpoint", targetRPM);
        //intakeMotor.setPower(shooterPIDFController.calculate(getVelocity(), targetRPM) + shooterFeedforward.calculate(getVelocity()));
    }

    public void setOpenLoopSetpoint(double speed) {
        telemetry.addData("ShooterOpenLoopSetpoint", speed);
        intakeMotor.setPower(speed);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity() / IntakeConstants.intakeMotorCPR;
    }
}
