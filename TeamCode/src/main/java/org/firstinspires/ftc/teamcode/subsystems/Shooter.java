package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.lights.Headlight;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;


//TODO: Everything currently set as -1 is a placeholder value and needs to be changed.
public class Shooter extends SubsystemBase {
    private Servo hoodServo;
    private Servo blockerServo;
    private DcMotor shooterMotor;

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

    private Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, ShooterConstants.shooterMotorID);
        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.hoodServoID);
        blockerServo = hardwareMap.get(Servo.class, ShooterConstants.blockerServoID);

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
            case RUNNING:
                systemState = SystemState.RUNNING;
            case EXHAUSTING:
                systemState = SystemState.EXHAUSTING;
            default:
                systemState = SystemState.IDLE;
        }

        return systemState;
    }

    private void applyStates() {
        switch (systemState) {
            case IDLE:
                shooterMotor.setPower(0.0);
                hoodServo.setPosition(ShooterConstants.hoodIdlePosition);
                blockerServo.setPosition(ShooterConstants.blockerIdlePosition);
            case READY:
                shooterMotor.setPower(
                        shooterPIDFController.calculate(-1, ShooterConstants.shooterReadyRPM) + shooterFeedforward.calculate(-1)
                );
        }
    }
}