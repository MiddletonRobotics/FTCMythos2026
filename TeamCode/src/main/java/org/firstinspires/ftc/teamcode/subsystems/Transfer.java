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

    private RevColorSensorV3 firstColorSensor;
    private RevColorSensorV3 secondColorSensor;

    private DigitalChannel firstBeamBreak;
    private DigitalChannel secondBeamBreak;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    private boolean isBlockerEngaged = true;
    private boolean isKickerEngaged = false;

    public Transfer(HardwareMap hMap, TelemetryManager telemetryM) {
        kickerServo = hMap.get(Servo.class, TransferConstants.kickerServoID);
        blockerServo = hMap.get(Servo.class, TransferConstants.blockerServoID);

        firstColorSensor = hMap.get(RevColorSensorV3.class, TransferConstants.firstColorSensorID);
        secondColorSensor = hMap.get(RevColorSensorV3.class, TransferConstants.secondColorSensorID);
        firstColorSensor.enableLed(true);
        secondColorSensor.enableLed(true);

        kickerServo.setDirection(Servo.Direction.REVERSE);

        firstBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.firstBeamBreakID);
        secondBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.secondBeamBreakID);

        firstBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        secondBeamBreak.setMode(DigitalChannel.Mode.INPUT);


        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        telemetryM.addData(TransferConstants.kSubsystemName + "fBB Distance Reading", firstCSDistance());
        telemetryM.addData(TransferConstants.kSubsystemName + "sBB Distance Reading", secondCSDistance());
        telemetryM.addData(TransferConstants.kSubsystemName + "Kicker Position", kickerServo.getPosition());
        telemetryM.addData(TransferConstants.kSubsystemName + "Blocker Reading", blockerServo.getPosition());
        telemetryM.addData(TransferConstants.kSubsystemName + "fBB Status",  isFirstBeamBroken());
        telemetryM.addData(TransferConstants.kSubsystemName + "sBB Status", isSecondBeamBroken());

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
    }



    public void setKickerPosition(double position) {
        telemetryM.addData(TransferConstants.kSubsystemName + "Kicker Target Position", position);
        telemetryM.addData(TransferConstants.kSubsystemName + "Kicker Current Position", Double.POSITIVE_INFINITY);
        kickerServo.setPosition(position);
    }

    public void setBlockerPosition(double position) {
        telemetryM.addData(TransferConstants.kSubsystemName + "BlockerTarget Position", position);
        telemetryM.addData(TransferConstants.kSubsystemName + "Blocker Current Position", Double.POSITIVE_INFINITY);
        blockerServo.setPosition(position);
    }

    public double firstCSDistance() {
        return firstColorSensor.getDistance(DistanceUnit.INCH);
    }

    public double secondCSDistance() {
        return secondColorSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean isBlockerEngaged() {
        return isBlockerEngaged;
    }

    public boolean isKickerEngaged() {return isKickerEngaged; }

    public boolean isFirstBeamBroken() {
        return !firstBeamBreak.getState();
    }

    public boolean isSecondBeamBroken() {
        return !secondBeamBreak.getState();
    }
}
