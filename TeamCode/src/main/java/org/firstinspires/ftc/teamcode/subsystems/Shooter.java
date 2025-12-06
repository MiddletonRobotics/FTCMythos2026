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
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    private Servo hoodServo;
    private DcMotorEx shooterMotor;

    private PIDFController shooterPIDFController;
    private SimpleMotorFeedforward shooterFeedforward;

    private static Shooter instance;
    public static Shooter getInstance(HardwareMap hMap, Telemetry telemetry) {
        if(instance == null) {
            instance = new Shooter(hMap, telemetry);
        }

        return instance;
    }

    private double hoodPosition = ShooterConstants.hoodIdlePosition;

    private Telemetry telemetry;
    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterMotorID);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.hoodServoID);

        shooterPIDFController = new PIDFController(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D, 0.0);
        shooterFeedforward = new SimpleMotorFeedforward(0.04, 0, 0);

        this.telemetry = telemetry;
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void onInitialization() {
        shooterMotor.setPower(0.0);
        hoodServo.setPosition(ShooterConstants.hoodIdlePosition);
    }

    @Override
    public void periodic() {
        telemetryManager.update();
    }

    public void setVelocitySetpoint(double targetRPM) {
        telemetryManager.addData("ShooterVelocitySetpoint", targetRPM);
        telemetryManager.addData("ShooterVelocityCurrent", getVelocity());
        telemetry.addData("ShooterVelocitySetpoint", targetRPM);
        telemetry.addData("ShooterVelocityCurrent", getVelocity());
        shooterMotor.setPower(shooterPIDFController.calculate(getVelocity(), targetRPM) + shooterFeedforward.calculate(getVelocity()));
    }

    public void setOpenLoopSetpoint(double speed) {
        telemetry.addData("ShooterOpenLoopSetpoint", speed);
        shooterMotor.setPower(speed);
    }

    public void setHoodPosition(double position) {
        this.hoodPosition = position;
    }

    public double getHoodTargetPosition() {
        return hoodServo.getPosition();
    }

    public double getVelocity() {
        return (shooterMotor.getVelocity() / ShooterConstants.shooterMotorCPR) * 60;
    }
}