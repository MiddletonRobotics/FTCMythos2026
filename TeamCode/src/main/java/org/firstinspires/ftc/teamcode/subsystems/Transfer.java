package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private Servo kickerServo;
    private Servo blockerServo;

    private DigitalChannel firstBeamBreak;
    private DigitalChannel secondBeamBreak;
    private DigitalChannel thridBeamBreak;

    public static Transfer instance;
    public static synchronized Transfer getInstance(HardwareMap hMap, TelemetryManager telemetryManager) {
        if(instance == null) {
            instance = new Transfer(hMap, telemetryManager);
        }

        return instance;
    }

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private double kickerPosition = TransferConstants.kickerIdlePosition;
    private double blockerPosition = TransferConstants.blockerIdlePosition;

    private Transfer(HardwareMap hMap, TelemetryManager telemetryManager) {
        kickerServo = hMap.get(Servo.class, TransferConstants.kickerServoID);
        blockerServo = hMap.get(Servo.class, TransferConstants.blockerServoID);

        firstBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.firstBeamBreakID);
        secondBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.secondBeamBreakID);
        thridBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.thirdBeamBreakID);

        firstBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        secondBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        thridBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        kickerServo.setDirection(Servo.Direction.REVERSE);

        this.telemetryManager = telemetryManager;
    }

    @Override
    public void periodic() {

    }

    public void setKickerPosition(double position) {
        kickerServo.setPosition(position);
    }

    public void setBlockerPosition(double position) {
        blockerServo.setPosition(position);
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
