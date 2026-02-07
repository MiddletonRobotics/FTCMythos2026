package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.utilities.Debouncer;
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

    private final Debouncer firstSensorDebouncer;
    private final Debouncer secondSensorDebouncer;
    private final Debouncer thirdSensorDebouncer;

    private boolean prevFirstDebounced = false;
    private boolean prevSecondDebounced = false;
    private boolean prevThirdDebounced = false;

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

        secondBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.firstBeamBreakID);
        thirdBeamBreak = hMap.get(DigitalChannel.class, TransferConstants.secondBeamBreakID);

        secondBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        thirdBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        firstSensorDebouncer = new Debouncer(TransferConstants.kSensorDebounceTime, Debouncer.DebounceType.Both);
        secondSensorDebouncer = new Debouncer(TransferConstants.kSensorDebounceTime, Debouncer.DebounceType.Both);
        thirdSensorDebouncer = new Debouncer(TransferConstants.kSensorDebounceTime, Debouncer.DebounceType.Both);

        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        boolean firstRaw = isFirstBeamBreakBroken();
        boolean secondRaw = isSecondBeamBroken();
        boolean thirdRaw = isThirdBeamBroken();

        boolean firstDebounced = firstSensorDebouncer.calculate(firstRaw);
        boolean secondDebounced = secondSensorDebouncer.calculate(secondRaw);
        boolean thirdDebounced = thirdSensorDebouncer.calculate(thirdRaw);

        setCurrentNumberOfBalls((firstDebounced ? 1 : 0) + (secondDebounced ? 1 : 0) + (thirdDebounced ? 1 : 0));
        //trackBallMovement(firstDebounced, secondDebounced, thirdDebounced);

        prevFirstDebounced = firstDebounced;
        prevSecondDebounced = secondDebounced;
        prevThirdDebounced = thirdDebounced;

        telemetryM.addData(TransferConstants.kSubsystemName + "fCS Raw Distance Reading", firstCSDistance());
        telemetryM.addData(TransferConstants.kSubsystemName + "Ball Count", (firstDebounced ? 1 : 0) + (secondDebounced ? 1 : 0) + (thirdDebounced ? 1 : 0));
        telemetryM.addData(TransferConstants.kSubsystemName + "Debounced States", String.format("F:%b S:%b T:%b", firstDebounced, secondDebounced, thirdDebounced));
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

        boolean firstBroken = isFirstBeamBreakBroken();
        boolean secondBroken = isSecondBeamBroken();
        boolean thirdBroken = isThirdBeamBroken();

        currentNumberOfBalls = 0;

        firstSensorDebouncer.reset(firstBroken);
        secondSensorDebouncer.reset(secondBroken);
        thirdSensorDebouncer.reset(thirdBroken);

        prevFirstDebounced = firstBroken;
        prevSecondDebounced = secondBroken;
        prevThirdDebounced = thirdBroken;
    }

    private void trackBallMovement(boolean first, boolean second, boolean third) {
        if (first && !prevFirstDebounced) {
            onBallEntered();
        }

        if (!third && prevThirdDebounced) {
            onBallExited();
        }

        validateBallCount(first, second, third);
    }

    private void onBallEntered() {
        currentNumberOfBalls = Math.min(currentNumberOfBalls + 1, 3);
    }

    private void onBallExited() {
        currentNumberOfBalls = Math.max(currentNumberOfBalls - 1, 0);
    }

    @SuppressLint("DefaultLocale")
    private void validateBallCount(boolean first, boolean second, boolean third) {
        int sensorCount = (first ? 1 : 0) + (second ? 1 : 0) + (third ? 1 : 0);

        // If there's a large discrepancy, log a warning
        if (Math.abs(sensorCount - currentNumberOfBalls) > 1) {
            telemetryM.addData(TransferConstants.kSubsystemName + "Warning", String.format("Count mismatch! Tracked: %d, Sensors: %d", currentNumberOfBalls, sensorCount));
        }
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

    public boolean doesTransferContainSingleBall() {
        return currentNumberOfBalls == 1;
    }

    public boolean doesTransferContainAnyBalls() {
        return currentNumberOfBalls > 0;
    }

    public boolean doesTransferContainAllBalls() {
        return currentNumberOfBalls == 3;
    }

    private double firstCSDistance() {
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
