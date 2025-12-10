package org.firstinspires.ftc.teamcode.subsystems;

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
    private Servo hoodServo;
    private DcMotorEx shooterMotor;

    private PIDFController velocityPIDFController;
    private SimpleMotorFeedforward velocityFeedforward;

    private InterpLUT shooterInteroperableMap = new InterpLUT();
    private InterpLUT hoodInteroperableMap = new InterpLUT();

    private Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterMotorID);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.hoodServoID);

        velocityPIDFController = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
        velocityFeedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

        this.telemetry = telemetry;
    }

    public void onInitialization() {
        shooterMotor.setPower(0.0);
        hoodServo.setPosition(ShooterConstants.hoodIdlePosition);
    }

    @Override
    public void periodic() {
        telemetry.addData(ShooterConstants.kSubsystemName + "Current Open Loop", getCurrentPower());
        telemetry.addData(ShooterConstants.kSubsystemName + "Current Velocity", getVelocity());
    }

    public void setVelocitySetpoint(double targetRPM) {
        telemetry.addData(ShooterConstants.kSubsystemName + "Velocity Setpoint", targetRPM);
        telemetry.addData(ShooterConstants.kSubsystemName + "Velocity Error", velocityPIDFController.getPositionError());
        telemetry.addData(ShooterConstants.kSubsystemName + "At Setpoint", velocityPIDFController.atSetPoint());

        if(GlobalConstants.kTuningMode) {
            velocityPIDFController.setPIDF(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
            velocityFeedforward.setCoefficient(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
        }

        shooterMotor.setPower(velocityPIDFController.calculate(getVelocity(), targetRPM) + velocityFeedforward.calculate(targetRPM));
    }

    public void setOpenLoopSetpoint(double speed) {
        telemetry.addData(ShooterConstants.kSubsystemName + "Setpoint Open Loop", speed);
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