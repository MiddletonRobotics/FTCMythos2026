package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private Servo kickerServo;
    private Servo blockerServo;

    private DigitalChannel firstBeamBreak;
    private DigitalChannel secondBeamBreak;
    private DigitalChannel thridBeamBreak;

    public enum SystemState {
        IDLE,
        BLOCK_FROM_SHOOTER,
        ALLOW_TO_SHOOTER,
        FEED_TO_SHOOTER,
    }

    public enum WantedState {
        IDLE,
        BLOCK_FROM_SHOOTER,
        ALLOW_TO_SHOOTER,
        FEED_TO_SHOOTER,
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public static Transfer instance;
    public static synchronized Transfer getInstance(HardwareMap hMap) {
        if(instance == null) {
            instance = new Transfer(hMap);
        }

        return instance;
    }

    private Transfer(HardwareMap hMap) {
        kickerServo = hMap.get(Servo.class, TransferConstants.kickerServoID);
        blockerServo = hMap.get(Servo.class, TransferConstants.blockerServoID);

        firstBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.firstBeamBreakID);
        secondBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.secondBeamBreakID);
        thridBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.thirdBeamBreakID);

        firstBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        secondBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        thridBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        blockerServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        systemState = handleTransition();
        applyStates();
    }

    private SystemState handleTransition() {
        switch (wantedState) {
            case IDLE:
                systemState = SystemState.IDLE;
                break;
            case FEED_TO_SHOOTER:
                systemState = SystemState.FEED_TO_SHOOTER;
                break;
            case ALLOW_TO_SHOOTER:
                systemState = SystemState.ALLOW_TO_SHOOTER;
                break;
            case BLOCK_FROM_SHOOTER:
                systemState = SystemState.BLOCK_FROM_SHOOTER;
                break;
        }

        return systemState;
    }

    private void applyStates() {
        switch (systemState) {
            case IDLE:
                blockerServo.setPosition(TransferConstants.blockerIdlePosition);
                kickerServo.setPosition(TransferConstants.kickerIdlePosition);
                break;
            case BLOCK_FROM_SHOOTER:
                blockerServo.setPosition(TransferConstants.blockerIdlePosition);
            case ALLOW_TO_SHOOTER:
                blockerServo.setPosition(TransferConstants.blockerAllowPosition);
            case FEED_TO_SHOOTER:
                blockerServo.setPosition(TransferConstants.blockerAllowPosition);
                kickerServo.setPosition(TransferConstants.kickerFeedPosition);
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean doesIntakeHaveBalls() {
        return !firstBeamBreak.getState() || !secondBeamBreak.getState() || !thridBeamBreak.getState();
    }

    public boolean isIntakeAtMaximumCapacity() {
        return !firstBeamBreak.getState() && !secondBeamBreak.getState() && !thridBeamBreak.getState();
    }

    public boolean isIntakePartiallyFull() {
        return doesIntakeHaveBalls() != isIntakeAtMaximumCapacity();
    }
}
