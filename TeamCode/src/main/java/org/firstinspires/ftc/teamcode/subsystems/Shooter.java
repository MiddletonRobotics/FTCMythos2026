package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    private Servo hoodServo;
    private DcMotorEx shooterMotor;

    private PIDFController velocityPIDFController;
    private SimpleMotorFeedforward velocityFeedforward;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterMotorID);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.hoodServoID);

        velocityPIDFController = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
        velocityFeedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

        this.telemetryManager = telemetryManager;
    }

    public void onInitialization() {
        shooterMotor.setPower(0.0);
        hoodServo.setPosition(ShooterConstants.hoodIdlePosition);
    }

    @Override
    public void periodic() {
        telemetryManager.addData(ShooterConstants.kSubsystemName + "Motor Power", getCurrentPower());
        telemetryManager.addData(ShooterConstants.kSubsystemName + "Current Velocity", getVelocity());
    }

    public void setVelocitySetpoint(double targetRPM) {
        telemetryManager.addData(ShooterConstants.kSubsystemName + "Velocity Setpoint", targetRPM);
        telemetryManager.addData(ShooterConstants.kSubsystemName + "Velocity Error", velocityPIDFController.getPositionError());
        telemetryManager.addData(ShooterConstants.kSubsystemName + "At Setpoint", velocityPIDFController.atSetPoint());

        //if(GlobalConstants.kTuningMode) {
            velocityPIDFController.setPIDF(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
        //}

        shooterMotor.setPower(velocityPIDFController.calculate(getVelocity(), targetRPM) + velocityFeedforward.calculate(getVelocity()));
    }

    public void setOpenLoopSetpoint(double speed) {
        telemetryManager.addData(ShooterConstants.kSubsystemName + "Open Loop", speed);
        shooterMotor.setPower(speed);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodTargetPosition() {
        return hoodServo.getPosition();
    }

    public double getCurrentPower() {
        return shooterMotor.getPower();
    }

    public double getVelocity() {
        return (shooterMotor.getVelocity() / ShooterConstants.shooterMotorCPR) * 60;
    }
}