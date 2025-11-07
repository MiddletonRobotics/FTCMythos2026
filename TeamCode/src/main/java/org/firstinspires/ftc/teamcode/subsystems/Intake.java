package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private MotorEx intakeMotor;
    private Motor.Encoder intakeEncoder;

    public enum SystemState {
        IDLE,
        RUNNING,
        EXHAUSTING
    }

    public enum WantedState {
        IDLE,
        RUNNING,
        EXHAUSTING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public static Intake instance;
    public static synchronized Intake getInstance(HardwareMap hMap) {
        if(instance == null) {
            instance = new Intake(hMap);
        }

        return instance;
    }

    private Intake(HardwareMap hMap) {
        intakeMotor = new MotorEx(hMap, IntakeConstants.intakeMotorID, Motor.GoBILDA.RPM_435);
        intakeMotor.setInverted(true);

        intakeEncoder = intakeMotor.encoder;
    }

    public void getVelocity() {
        intakeEncoder.getCorrectedVelocity();
    }

    @Override
    public void periodic() {
        systemState = handleTransition();
        applyStates();
    }

    public boolean currentlyHoldingBalls() {
        return false;
    }

    private SystemState handleTransition() {
        switch(wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case EXHAUSTING:
                return SystemState.EXHAUSTING;
            case RUNNING:
                return SystemState.RUNNING;
            default:
                return SystemState.IDLE;
        }
    }

    private void applyStates() {
        switch (systemState) {
            case IDLE:

        }
    }

    public void setIntakeTargetRPM(double rpm) {
        double currSpeed = rpm / intakeMotor.getMaxRPM();
        intakeMotor.set(currSpeed);
        wantedState = WantedState.RUNNING;
    }
}
