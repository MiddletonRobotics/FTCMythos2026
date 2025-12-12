package org.firstinspires.ftc.teamcode.subsystems;

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
    private Servo LED;
    private Telemetry telemetry;

    private Timing.Timer ledTimer;
    private long intervalMs = 0;
    private LEDConstants.ColorValue storedColor;

    public boolean isBlinking = false;
    private boolean blinkStateOn = false;

    public LED(HardwareMap hMap, Telemetry telemetry) {
        LED = hMap.get(Servo.class, LEDConstants.kLedServoID);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData(LEDConstants.kSubsystemName + "Current Color", LED.getPosition());
    }

    public void onInitialization(GlobalConstants.AllianceColor allianceColor) {
        if(allianceColor == GlobalConstants.AllianceColor.BLUE) {
            enableSolidColor(LEDConstants.ColorValue.BLUE);
        } else {
            enableSolidColor(LEDConstants.ColorValue.RED);
        }
    }

    public void enableSolidColor(LEDConstants.ColorValue colorValue) {
        this.isBlinking = false;
        this.storedColor = colorValue;
    }

    public void enableBlinking(long intervalMs, LEDConstants.ColorValue colorValue) {
        this.intervalMs = intervalMs;
        this.storedColor = colorValue;

        ledTimer = new Timing.Timer(intervalMs, TimeUnit.MILLISECONDS);
        ledTimer.start();

        isBlinking = true;
        blinkStateOn = false;
        setPosition(storedColor.getColorPosition());
    }

    public void update() {
        if (!isBlinking) setPosition(storedColor.getColorPosition());

        if (ledTimer.done()) {
            blinkStateOn = !blinkStateOn;
            ledTimer.start();

            setPosition(blinkStateOn ? storedColor.getColorPosition() : LEDConstants.ColorValue.OFF.getColorPosition());
        }
    }

    public void stopBlinking() {
        isBlinking = false;
        setPosition(LEDConstants.ColorValue.OFF.getColorPosition());
    }

    public void setPosition(double position){
        LED.setPosition(position);
    }
}