package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

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

    private double targetRPM = ShooterConstants.shooterReadyRPM;

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

    }

    public void setShooterRPM(double targetRPM) {
        shooterMotor.setPower(targetRPM / 6000);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodTargetPosition() {
        return hoodServo.getPosition();
    }

    public double getVelocity() {
        return shooterMotor.getVelocity() / ShooterConstants.shooterMotorCPR;
    }
}