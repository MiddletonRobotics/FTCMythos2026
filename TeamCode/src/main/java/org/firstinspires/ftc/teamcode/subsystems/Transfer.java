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

    public Transfer(HardwareMap hMap, TelemetryManager telemetryManager) {
        kickerServo = hMap.get(Servo.class, TransferConstants.kickerServoID);
        blockerServo = hMap.get(Servo.class, TransferConstants.blockerServoID);

        firstBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.firstBeamBreakID);
        secondBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.secondBeamBreakID);
        thridBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.thirdBeamBreakID);

        colorSensor = hMap.get(RevColorSensorV3.class, TransferConstants.colorSensorID);
        colorSensor.enableLed(true);

        firstBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        secondBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        thridBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        kickerServo.setDirection(Servo.Direction.REVERSE);

        this.telemetryManager = telemetryManager;
    }

    @Override
    public void periodic() {
        telemetryManager.addData(TransferConstants.kSubsystemName + "tBB Distance Reading", colorSensor.getDistance(DistanceUnit.INCH));
    }

    public void onInitialization(boolean initKicker, boolean initBlocker) {
        if(initKicker) kickerServo.setPosition(TransferConstants.kickerIdlePosition);
        if(initBlocker) blockerServo.setPosition(TransferConstants.blockerIdlePosition);
    }

    public void setKickerPosition(double position) {
        telemetryManager.addData(TransferConstants.kSubsystemName + "Kicker Target Position", position);
        telemetryManager.addData(TransferConstants.kSubsystemName + "Kicker Current Position", Double.POSITIVE_INFINITY);
        kickerServo.setPosition(position);
    }

    public void setBlockerPosition(double position) {
        telemetryManager.addData(TransferConstants.kSubsystemName + "BlockerTarget Position", position);
        telemetryManager.addData(TransferConstants.kSubsystemName + "Blocker Current Position", Double.POSITIVE_INFINITY);
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
