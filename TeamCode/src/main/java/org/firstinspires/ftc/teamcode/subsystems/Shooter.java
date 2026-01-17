package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.firstinspires.ftc.library.utilities.InterpLUT;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final Servo hoodServo;
    private final DcMotorEx shooterMotor;

    private final PIDFController velocityPIDFController;
    private final SimpleMotorFeedforward velocityFeedforward;

    private final InterpLUT shooterInteroperableMap = new InterpLUT();
    private final InterpLUT hoodInteroperableMap = new InterpLUT();

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetryM) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterMotorID);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.hoodServoID);

        shooterInteroperableMap.add(460.0, 3500);
        shooterInteroperableMap.add(578.0, 3600);
        shooterInteroperableMap.add(675.2, 3800);
        shooterInteroperableMap.add(710.5, 4100);
        shooterInteroperableMap.add(782.3, 4150);
        shooterInteroperableMap.createLUT();

        hoodInteroperableMap.add(460.0, 0.4);
        hoodInteroperableMap.add(578.0, 0.41);
        hoodInteroperableMap.add(675.2, 0.43);
        hoodInteroperableMap.add(710.5, 0.44);
        hoodInteroperableMap.add(782.3, 0.445);
        hoodInteroperableMap.createLUT();

        velocityPIDFController = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
        velocityFeedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

        velocityPIDFController.setTolerance(100);

        this.telemetryM = telemetryM;
    }

    public void onInitialization() {
        shooterMotor.setPower(0.0);
        hoodServo.setPosition(ShooterConstants.hoodIdlePosition);
    }

    @Override
    public void periodic() {
        telemetryM.addData(ShooterConstants.kSubsystemName + "Current Open Loop", getCurrentPower());
        telemetryM.addData(ShooterConstants.kSubsystemName + "Current Velocity", getVelocity());
    }

    public void setVelocitySetpoint(double targetRPM) {
        telemetryM.addData(ShooterConstants.kSubsystemName + "Velocity Setpoint", targetRPM);
        telemetryM.addData(ShooterConstants.kSubsystemName + "Velocity Error", velocityPIDFController.getPositionError());
        telemetryM.addData(ShooterConstants.kSubsystemName + "At Setpoint", velocityPIDFController.atSetPoint());

        if(GlobalConstants.kTuningMode) {
            velocityPIDFController.setPIDF(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
            velocityFeedforward.setCoefficient(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
        }

        shooterMotor.setPower(velocityPIDFController.calculate(getVelocity(), targetRPM) + velocityFeedforward.calculate(targetRPM));
    }

    public boolean flywheelAtSetpoint() {
        return velocityPIDFController.atSetPoint();
    }

    public void setOpenLoopSetpoint(double speed) {
        telemetryM.addData(ShooterConstants.kSubsystemName + "Setpoint Open Loop", speed);
        shooterMotor.setPower(speed);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodTargetPosition() {
        return hoodServo.getPosition();
    }

    public double calculateFlywheelSpeeds(double distance) {
        return shooterInteroperableMap.get(distance);
    }

    public double calculateHoodPosition(double distance) {
        return hoodInteroperableMap.get(distance);
    }

    public double getCurrentPower() {
        return shooterMotor.getPower();
    }

    public double getVelocity() {
        return (shooterMotor.getVelocity() / ShooterConstants.shooterMotorCPR) * 60;
    }
}