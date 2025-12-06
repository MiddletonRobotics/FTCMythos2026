package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private Servo kickerServo;
    private Servo blockerServo;

    private DigitalChannel firstBeamBreak;
    private DigitalChannel secondBeamBreak;
    private DigitalChannel thridBeamBreak;

    private RevColorSensorV3 colorSensor;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    public static Transfer instance;
    public static Transfer getInstance(HardwareMap hMap, TelemetryManager telemetryManager) {
        if(instance == null) {
            instance = new Transfer(hMap, telemetryManager);
        }

        return instance;
    }

    private Transfer(HardwareMap hMap, TelemetryManager telemetryManager) {
        kickerServo = hMap.get(Servo.class, TransferConstants.kickerServoID);
        blockerServo = hMap.get(Servo.class, TransferConstants.blockerServoID);

        firstBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.firstBeamBreakID);
        secondBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.secondBeamBreakID);
        thridBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.thirdBeamBreakID);

        colorSensor = hMap.get(RevColorSensorV3.class, TransferConstants.colorSensorID);

        firstBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        secondBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        thridBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        kickerServo.setDirection(Servo.Direction.REVERSE);

        this.telemetryManager = telemetryManager;
    }

    public void onInitialization(boolean initKicker, boolean initBlocker) {
        if(initKicker) kickerServo.setPosition(TransferConstants.kickerIdlePosition);
        if(initBlocker) blockerServo.setPosition(TransferConstants.blockerIdlePosition);
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

    public boolean isBallCurrentlyStaged() {
        return colorSensor.getDistance(DistanceUnit.INCH) < 2;
    }

    public boolean isIntakePartiallyFull() {
        return doesIntakeHaveBalls() != isIntakeAtMaximumCapacity();
    }
}
