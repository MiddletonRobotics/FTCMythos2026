package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.lights.LightsManager;
import com.bylazar.lights.PanelsLights;
import com.bylazar.lights.RGBIndicator;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.utilities.Timing;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;

import java.util.concurrent.TimeUnit;

public class LED extends SubsystemBase {
    private Servo ledServo;
    private Timing.Timer ledTimer = new Timing.Timer(0, TimeUnit.MILLISECONDS);

    private LedState mode = LedState.OFF;

    private LEDConstants.ColorValue primaryColorA = LEDConstants.ColorValue.OFF;
    private LEDConstants.ColorValue primaryColorB = LEDConstants.ColorValue.OFF;
    private LEDConstants.ColorValue secondaryColorA = LEDConstants.ColorValue.ORANGE;
    private LEDConstants.ColorValue secondaryColorB = LEDConstants.ColorValue.ORANGE;

    private LEDConstants.ColorValue currentColor = LEDConstants.ColorValue.OFF;

    private int patternStep = 0;
    private long intervalMs = 200;

    @IgnoreConfigurable
    static LightsManager lightsManager;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    private RGBIndicator ledIndicator;

    public enum LedState {
        OFF,
        SOLID,
        BLINK_SIMPLE,
        BLINK_PATTERN
    }

    public LED(HardwareMap hMap, TelemetryManager telemetryM, LightsManager lightsManager) {
        this.ledServo = hMap.get(Servo.class, LEDConstants.kLedServoID);
        this.telemetryM = telemetryM;
        this.lightsManager = lightsManager;

        this.ledIndicator = new RGBIndicator("Colored Light #2");
        lightsManager.initLights(ledIndicator);
    }


    @Override
    public void periodic() {
        telemetryM.addData(LEDConstants.kSubsystemName + "Pattern Step", patternStep);
        ledIndicator.update(currentColor.getColorPosition());
        lightsManager.update();

        update();
    }

    private void setColor(LEDConstants.ColorValue color) {
        this.currentColor = color;
        ledServo.setPosition(color.getColorPosition());
    }

    public void setSolid(LEDConstants.ColorValue color) {
        mode = LedState.SOLID;
        primaryColorA = color;

        if (ledTimer.isTimerOn()) {
            ledTimer.pause();
        }

        patternStep = 0;
    }

    public void setSimpleBlink(LEDConstants.ColorValue colorA, LEDConstants.ColorValue colorB, long intervalMs) {
        mode = LedState.BLINK_SIMPLE;

        primaryColorA = colorA;
        primaryColorB = colorB;

        resetTimer(intervalMs);
    }

    public void setDefaultSimpleBlink(LEDConstants.ColorValue color, long intervalMs) {
        setSimpleBlink(color, LEDConstants.ColorValue.OFF, intervalMs);
    }


    public void setComplexBlink(LEDConstants.ColorValue allianceColor, LEDConstants.ColorValue accentColor, LEDConstants.ColorValue idleColor, long intervalMs) {
        mode = LedState.BLINK_PATTERN;

        primaryColorA = allianceColor;
        primaryColorB = idleColor;
        secondaryColorA = accentColor;
        secondaryColorB = idleColor;

        this.intervalMs = intervalMs;
        patternStep = 0;

        ledTimer = new Timing.Timer(intervalMs, TimeUnit.MILLISECONDS);
        ledTimer.start();
    }

    public void setDefaultComplexBlink(LEDConstants.ColorValue allianceColor, LEDConstants.ColorValue accentColor, long intervalMs) {
        setComplexBlink(allianceColor, accentColor, LEDConstants.ColorValue.OFF, intervalMs);
    }

    public void update() {
        switch (mode) {
            case SOLID:
                setColor(primaryColorA);
                break;
            case BLINK_SIMPLE:
                if (ledTimer.done()) {
                    ledTimer.start();
                    patternStep ^= 1;
                }

                setColor(patternStep == 0 ? primaryColorA : primaryColorB);
                break;
            case BLINK_PATTERN:
                if (ledTimer.done()) {
                    ledTimer.start();
                    patternStep = (patternStep + 1) % 4;
                }

                switch (patternStep) {
                    case 0: setColor(primaryColorA); break;
                    case 1: setColor(primaryColorB); break;
                    case 2: setColor(secondaryColorA); break;
                    case 3: setColor(secondaryColorB); break;
                }
                break;
            case OFF:
                if (ledTimer.isTimerOn()) {
                    ledTimer.pause();
                    ledTimer = new Timing.Timer(0, TimeUnit.MILLISECONDS); // Reset
                }

                patternStep = 0;
                setColor(LEDConstants.ColorValue.OFF);
                break;
            default:
                setColor(LEDConstants.ColorValue.OFF);
                break;
        }
    }

    private void resetTimer(long intervalMs) {
        this.intervalMs = intervalMs;
        ledTimer = new Timing.Timer(intervalMs, TimeUnit.MILLISECONDS);
        ledTimer.start();
        patternStep = 0;
    }
}