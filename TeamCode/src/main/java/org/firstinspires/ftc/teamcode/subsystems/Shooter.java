package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    private Servo hoodServo;
    private DcMotorEx shooterMotor;

    private PIDFController shooterPIDFController;
    private SimpleMotorFeedforward shooterFeedforward;

    private static Shooter instance;
    public static synchronized Shooter getInstance(HardwareMap hMap, TelemetryManager telemetryManager) {
        if(instance == null) {
            instance = new Shooter(hMap, telemetryManager);
        }

        return instance;
    }

    private double hoodPosition = ShooterConstants.hoodIdlePosition;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private Shooter(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterMotorID);
        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.hoodServoID);

        shooterPIDFController = new PIDFController(0.01, 0, 0.0, 0.0);
        shooterFeedforward = new SimpleMotorFeedforward(0, 0, 0);

        this.telemetryManager = telemetryManager;
    }

    @Override
    public void periodic() {
        hoodServo.setPosition(hoodPosition);
    }

    public void setVelocitySetpoint(double targetRPM) {
        telemetryManager.addData("ShooterVelocitySetpoint", targetRPM);
        shooterMotor.setPower(shooterPIDFController.calculate(getVelocity(), targetRPM) + shooterFeedforward.calculate(getVelocity()));
    }

    public void setOpenLoopSetpoint(double speed) {
        telemetryManager.addData("ShooterOpenLoopSetpoint", speed);
        shooterMotor.setPower(speed);
    }

    public Command velocitySetpointCommand(DoubleSupplier setpoint) {
        return Commands.runEnd(() -> {
            double velocity = setpoint.getAsDouble();
            setVelocitySetpoint(velocity);
        }, () -> {
            setOpenLoopSetpoint(0.0);
        }).withName("Shooter Velocity");
    }

    public Command openLoopSetpointCommand(DoubleSupplier setpoint) {
        return Commands.runEnd(() -> {
            setOpenLoopSetpoint(setpoint.getAsDouble());
        }, () -> {
            setOpenLoopSetpoint(0.0);
        }).withName("Shooter Open Loop");
    }

    public void setHoodPosition(double position) {
        this.hoodPosition = position;
    }

    public double getHoodTargetPosition() {
        return hoodServo.getPosition();
    }

    public double getVelocity() {
        return shooterMotor.getVelocity() / ShooterConstants.shooterMotorCPR * 60;
    }
}