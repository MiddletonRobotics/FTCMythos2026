package org.firstinspires.ftc.teamcode.subsystems;

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

    private Telemetry telemetry;

    public Transfer(HardwareMap hMap, Telemetry telemetry) {
        kickerServo = hMap.get(Servo.class, TransferConstants.kickerServoID);
        blockerServo = hMap.get(Servo.class, TransferConstants.blockerServoID);

        firstColorSensor = hMap.get(RevColorSensorV3.class, TransferConstants.firstColorSensorID);
        secondColorSensor = hMap.get(RevColorSensorV3.class, TransferConstants.secondColorSensorID);
        firstColorSensor.enableLed(true);
        secondColorSensor.enableLed(true);

        kickerServo.setDirection(Servo.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData(TransferConstants.kSubsystemName + "fBB Distance Reading", firstCSDistance());
        telemetry.addData(TransferConstants.kSubsystemName + "sBB Distance Reading", secondCSDistance());
    }

    public void onInitialization(boolean initKicker, boolean initBlocker) {
        if(initKicker) kickerServo.setPosition(TransferConstants.kickerIdlePosition);
        if(initBlocker) blockerServo.setPosition(TransferConstants.blockerIdlePosition);
    }

    public void setKickerPosition(double position) {
        telemetry.addData(TransferConstants.kSubsystemName + "Kicker Target Position", position);
        telemetry.addData(TransferConstants.kSubsystemName + "Kicker Current Position", Double.POSITIVE_INFINITY);
        kickerServo.setPosition(position);
    }

    public void setBlockerPosition(double position) {
        telemetry.addData(TransferConstants.kSubsystemName + "BlockerTarget Position", position);
        telemetry.addData(TransferConstants.kSubsystemName + "Blocker Current Position", Double.POSITIVE_INFINITY);
        blockerServo.setPosition(position);
    }

    public double firstCSDistance() {
        return firstColorSensor.getDistance(DistanceUnit.INCH);
    }

    public double secondCSDistance() {
        return secondColorSensor.getDistance(DistanceUnit.INCH);
    }
}
