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

    private final InterpLUT shooterInteroperableMap;
    private final InterpLUT hoodInteroperableMap;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetryM) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterMotorID);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.hoodServoID);

        shooterInteroperableMap = new InterpLUT()
            .add(20, 3200)
            .add(47.259, 3600)
            .add(63.864, 3800)
            .add(80.426, 4100)
            .add(96.217, 4300)
            .add(132.282, 4700)
            .add(148.92, 5000)
            .add(200, 5600)
            .createLUT();

        hoodInteroperableMap = new InterpLUT()
            .add(20, 1)
            .add(47.259, 0.685)
            .add(63.864, 0.45)
            .add(80.426, 0.3)
            .add(96.217, 0.2)
            .add(132.282, 0.0)
            .add(200, 0.0)
            .createLUT();

        velocityPIDFController = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
        velocityFeedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

        velocityPIDFController.setTolerance(100);

        this.telemetryM = telemetryM;
    }

    public void onInitialization() {
        shooterMotor.setPower(0.0);
        hoodServo.setPosition(ShooterConstants.kHoodMinimumPosition);
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