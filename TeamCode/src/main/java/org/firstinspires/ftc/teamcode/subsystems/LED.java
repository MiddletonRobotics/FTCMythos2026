package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
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
    private Servo LED;

    private Timing.Timer ledTimer = new Timing.Timer(0, TimeUnit.MILLISECONDS);
    private long intervalMs = 0;
    private LEDConstants.ColorValue storedColor = LEDConstants.ColorValue.OFF;

    public boolean isBlinking = false;
    private boolean blinkStateOn = false;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public LED(HardwareMap hMap, TelemetryManager telemetryM) {
        LED = hMap.get(Servo.class, LEDConstants.kLedServoID);
        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        telemetryM.addData(LEDConstants.kSubsystemName + "Current Color", LED.getPosition());
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

        if (ledTimer.done() && isBlinking) {
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