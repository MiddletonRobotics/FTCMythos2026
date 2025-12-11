package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    private TelemetryPacket shooterPacket;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterMotorID);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.hoodServoID);

        shooterInteroperableMap.add(437.5, 3600);
        shooterInteroperableMap.add(492.7, 3600);
        shooterInteroperableMap.add(552.8, 4100);
        shooterInteroperableMap.add(603.6, 4100);
        shooterInteroperableMap.add(702.3, 4200);
        shooterInteroperableMap.add(750.0, 4300);
        shooterInteroperableMap.add(870.0, 5600);
        shooterInteroperableMap.createLUT();

        hoodInteroperableMap.add(437.5, 0.45);
        hoodInteroperableMap.add(492.5, 0.47);
        hoodInteroperableMap.add(552.8, 0.5);
        hoodInteroperableMap.add(603.6, 0.48);
        hoodInteroperableMap.add(702.3, 0.5);
        hoodInteroperableMap.add(750.0, 0.52);
        hoodInteroperableMap.add(870, 0.36);
        hoodInteroperableMap.createLUT();

        velocityPIDFController = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
        velocityFeedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

        this.telemetry = telemetry;
        shooterPacket = new TelemetryPacket();
    }

    public void onInitialization() {
        shooterMotor.setPower(0.0);
        hoodServo.setPosition(ShooterConstants.hoodIdlePosition);
    }

    @Override
    public void periodic() {
        shooterPacket.put(ShooterConstants.kSubsystemName + "Current Open Loop", getCurrentPower());
        shooterPacket.put(ShooterConstants.kSubsystemName + "Current Velocity", getVelocity());

        FtcDashboard.getInstance().sendTelemetryPacket(shooterPacket);
    }

    public void setVelocitySetpoint(double targetRPM) {
        shooterPacket.put(ShooterConstants.kSubsystemName + "Velocity Setpoint", targetRPM);
        shooterPacket.put(ShooterConstants.kSubsystemName + "Velocity Error", velocityPIDFController.getPositionError());
        shooterPacket.put(ShooterConstants.kSubsystemName + "At Setpoint", velocityPIDFController.atSetPoint());

        if(GlobalConstants.kTuningMode) {
            velocityPIDFController.setPIDF(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
            velocityFeedforward.setCoefficient(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
        }

        shooterMotor.setPower(velocityPIDFController.calculate(getVelocity(), targetRPM) + velocityFeedforward.calculate(targetRPM));
    }

    public void setOpenLoopSetpoint(double speed) {
        shooterPacket.put(ShooterConstants.kSubsystemName + "Setpoint Open Loop", speed);
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