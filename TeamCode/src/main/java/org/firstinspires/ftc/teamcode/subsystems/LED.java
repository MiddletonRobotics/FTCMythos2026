package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;

public class LED extends SubsystemBase {
    private Servo LED;

    @IgnoreConfigurable
    private TelemetryManager telemetryManager;

    public LED(HardwareMap hMap, TelemetryManager telemetryManager) {
        LED = hMap.get(Servo.class, LEDConstants.kLedServoID);
        this.telemetryManager = telemetryManager;
    }

    public void onInitialization(GlobalConstants.AllianceColor allianceColor) {
        if(allianceColor == GlobalConstants.AllianceColor.BLUE) {
            setColor(LEDConstants.ColorValue.BLUE);
        } else {
            setColor(LEDConstants.ColorValue.RED);
        }
    }

    public void setColor(LEDConstants.ColorValue colorValue) {
        telemetryManager.addData(LEDConstants.kSubsystemName + "Current Color", colorValue.toString());
        telemetryManager.addData(LEDConstants.kSubsystemName + "Current Color Position", colorValue.getColorPosition());

        LED.setPosition(colorValue.getColorPosition());
    }
}
