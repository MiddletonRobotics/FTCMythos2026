package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;


//TODO: Everything currently set as -1 is a placeholder value and needs to be changed.
public class Shooter extends SubsystemBase {
    private Servo hoodServo;
    private DcMotorEx shooterMotor;

    public enum SystemState {
        IDLE,
        READY,
        RUNNING,
        EXHAUSTING
    }

    public enum WantedState {
        IDLE,
        READY,
        RUNNING,
        EXHAUSTING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private PIDFController shooterPIDFController;
    private SimpleMotorFeedforward shooterFeedforward;

    private static Shooter instance;
    public static synchronized Shooter getInstance(HardwareMap hMap) {
        if(instance == null) {
            instance = new Shooter(hMap);
        }

        return instance;
    }

    private double targetRPM = ShooterConstants.shooterReadyRPM;

    private Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterMotorID);
        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.hoodServoID);

        shooterPIDFController = new PIDFController(0.01, 0, 0.0, 0.0);
        shooterFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    }

    @Override
    public void periodic() {
        systemState = handleTransition();
        applyStates();
    }

    private SystemState handleTransition() {
        switch(wantedState) {
            case READY:
                systemState = SystemState.READY;
                break;
            case RUNNING:
                systemState = SystemState.RUNNING;
                break;
            case EXHAUSTING:
                systemState = SystemState.EXHAUSTING;
                break;
            default:
                systemState = SystemState.IDLE;
                break;
        }

        return systemState;
    }

    private void applyStates() {
        switch (systemState) {
            case IDLE:
                shooterMotor.setPower(0.0);
                hoodServo.setPosition(ShooterConstants.hoodIdlePosition);
                break;
            case READY:
                shooterMotor.setPower(
                        shooterPIDFController.calculate(shooterMotor.getVelocity(), ShooterConstants.shooterReadyRPM) + shooterFeedforward.calculate(shooterMotor.getVelocity())
                );

                break;
            case RUNNING:
                shooterMotor.setPower(
                        shooterPIDFController.calculate(shooterMotor.getVelocity(), targetRPM) + shooterFeedforward.calculate(shooterMotor.getVelocity())
                );

                break;
            case EXHAUSTING:
                shooterMotor.setPower(-0.2);
                hoodServo.setPosition(ShooterConstants.hoodIdlePosition);
                break;
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setShooterRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        setWantedState(WantedState.RUNNING);
    }

    public double getVelocity() {
        return shooterMotor.getVelocity() / ShooterConstants.shooterMotorCPR;
    }
}