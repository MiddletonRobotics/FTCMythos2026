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

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private PIDFController velocityPIDFController;
    private SimpleMotorFeedforward velocityFeedforward;

    public Intake(HardwareMap hMap, TelemetryManager telemetryManager) {
        intakeMotor = hMap.get(DcMotorEx.class, IntakeConstants.intakeMotorID);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityPIDFController = new PIDFController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kF);
        velocityFeedforward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);

        this.telemetryManager = telemetryManager;
    }

    @Override
    public void periodic() {
    }

    public void setVelocitySetpoint(double targetRPM) {
        telemetryManager.addData(IntakeConstants.kSubsystemName + "Setpoint Velocity", targetRPM);
        telemetryManager.addData(IntakeConstants.kSubsystemName + "Current Velocity", getVelocity());
        telemetryManager.addData(IntakeConstants.kSubsystemName + "Velocity Error", velocityPIDFController.getPositionError());
        telemetryManager.addData(IntakeConstants.kSubsystemName + "At Setpoint?", velocityPIDFController.atSetPoint());

        if(GlobalConstants.kTuningMode) {
            velocityPIDFController.setPIDF(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kF);
        }

        intakeMotor.setPower(velocityPIDFController.calculate(getVelocity(), targetRPM) + velocityFeedforward.calculate(getVelocity()));
    }

    public void setOpenLoopSetpoint(double speed) {
        telemetryManager.addData(IntakeConstants.kSubsystemName + "Open Loop", speed);
        intakeMotor.setPower(speed);
    }

    public double getVelocity() {
        return (intakeMotor.getVelocity() / IntakeConstants.intakeMotorCPR) * 60;
    }
}
