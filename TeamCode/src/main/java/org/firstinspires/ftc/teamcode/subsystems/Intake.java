package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private DcMotorEx intakeMotor;

    private PIDFController velocityPIDFController;
    private SimpleMotorFeedforward velocityFeedforward;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public Intake(HardwareMap hMap, TelemetryManager telemetryM) {
        intakeMotor = hMap.get(DcMotorEx.class, IntakeConstants.intakeMotorID);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityPIDFController = new PIDFController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kF);
        velocityFeedforward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);

        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        telemetryM.addData(IntakeConstants.kSubsystemName + "Current Velocity", getVelocity());
        telemetryM.addData(IntakeConstants.kSubsystemName + "Current Open Loop", intakeMotor.getPower());
    }

    public void setVelocitySetpoint(double targetRPM) {
        telemetryM.addData(IntakeConstants.kSubsystemName + "Setpoint Velocity", targetRPM);
        telemetryM.addData(IntakeConstants.kSubsystemName + "Velocity Error", velocityPIDFController.getPositionError());
        telemetryM.addData(IntakeConstants.kSubsystemName + "At Setpoint?", velocityPIDFController.atSetPoint());

        if(GlobalConstants.kTuningMode) {
            velocityPIDFController.setPIDF(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kF);
            velocityFeedforward.setCoefficient(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);
        }

        intakeMotor.setPower(velocityPIDFController.calculate(getVelocity(), targetRPM) + velocityFeedforward.calculate(targetRPM));
    }

    public void setOpenLoopSetpoint(double speed) {
        telemetryM.addData(IntakeConstants.kSubsystemName + "Setpoint Open Loop", speed);
        intakeMotor.setPower(speed);
    }

    public double getVelocity() {
        return (intakeMotor.getVelocity() / IntakeConstants.intakeMotorCPR) * 60;
    }
}
