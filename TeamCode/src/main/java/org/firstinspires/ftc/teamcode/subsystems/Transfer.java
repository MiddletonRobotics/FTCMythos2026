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

import lombok.Getter;
import lombok.Setter;

public class Transfer extends SubsystemBase {
    private final Servo kickerServo;
    private final Servo blockerServo;

    private final RevColorSensorV3 firstColorSensor;

    private final DigitalChannel secondBeamBreak;
    private final DigitalChannel thirdBeamBreak;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Getter
    @Setter
    private boolean isBlockerEngaged = true;

    @Getter
    @Setter
    private boolean isKickerEngaged = false;

    @Getter
    @Setter
    private int currentNumberOfBalls = 0;

    public Transfer(HardwareMap hMap, TelemetryManager telemetryM) {
        kickerServo = hMap.get(Servo.class, TransferConstants.kickerServoID);
        blockerServo = hMap.get(Servo.class, TransferConstants.blockerServoID);

        kickerServo.setDirection(Servo.Direction.REVERSE);
        blockerServo.setDirection(Servo.Direction.FORWARD);

        firstColorSensor = hMap.get(RevColorSensorV3.class, TransferConstants.firstColorSensorID);
        firstColorSensor.enableLed(true);

        secondBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.secondBeamBreakID);
        thirdBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.firstBeamBreakID);

        secondBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        thirdBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        telemetryM.addData(TransferConstants.kSubsystemName + "fCS Distance Reading", firstCSDistance());
        telemetryM.addData(TransferConstants.kSubsystemName + "sBB Status",  isSecondBeamBroken());
        telemetryM.addData(TransferConstants.kSubsystemName + "tBB Status", isThirdBeamBroken());

        telemetryM.addData(TransferConstants.kSubsystemName + "Kicker Position", kickerServo.getPosition());
        telemetryM.addData(TransferConstants.kSubsystemName + "Blocker Position", blockerServo.getPosition());
    }

    public void onInitialization(boolean initKicker, boolean initBlocker) {
        if(initKicker) {
            kickerServo.setPosition(TransferConstants.kickerIdlePosition);
            isKickerEngaged = false;
        }

        if(initBlocker) {
            blockerServo.setPosition(TransferConstants.blockerIdlePosition);
            isBlockerEngaged = true;
        }

        setCurrentNumberOfBalls((isFirstBeamBreakBroken() ? 1 : 0) + (isSecondBeamBroken() ? 1 : 0) + (isThirdBeamBroken() ? 1 : 0));
    }

    public void setKickerPosition(double position) {
        telemetryM.addData(TransferConstants.kSubsystemName + "Kicker Target Position", position);
        telemetryM.addData(TransferConstants.kSubsystemName + "Kicker Current Position", Double.POSITIVE_INFINITY);
        kickerServo.setPosition(position);
    }

    public void setBlockerPosition(double position) {
        telemetryM.addData(TransferConstants.kSubsystemName + "Blocker Target Position", position);
        telemetryM.addData(TransferConstants.kSubsystemName + "Blocker Current Position", Double.POSITIVE_INFINITY);
        blockerServo.setPosition(position);
    }

    public double firstCSDistance() {
        return firstColorSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean isFirstBeamBreakBroken() {
        return firstCSDistance() < TransferConstants.kFirstColorSensorDistanceThreshold;
    }

    public boolean isSecondBeamBroken() {
        return !secondBeamBreak.getState();
    }

    public boolean isThirdBeamBroken() {
        return !thirdBeamBreak.getState();
    }
}
