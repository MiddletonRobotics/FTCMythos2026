package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;

public class LED extends SubsystemBase {
    private Servo LED;

    public LED(HardwareMap hMap) {
        LED = hMap.get(Servo.class, LEDConstants.kLedServoID);
    }

    public void onInitialization(GlobalConstants.AllianceColor allianceColor) {
        if(allianceColor == GlobalConstants.AllianceColor.BLUE) {
            setColor(LEDConstants.ColorValue.BLUE);
        } else {
            setColor(LEDConstants.ColorValue.RED);
        }
    }

    public void setColor(LEDConstants.ColorValue colorValue) {
        //telemetryManager.addData(LEDConstants.kSubsystemName + "Current Color", colorValue.toString());
        //telemetryManager.addData(LEDConstants.kSubsystemName + "Current Color Position", colorValue.getColorPosition());

        LED.setPosition(colorValue.getColorPosition());
    }
}
