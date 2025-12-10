package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;

public class LED extends SubsystemBase {
    private Servo LED;
    private Telemetry telemetry;

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
            setColor(LEDConstants.ColorValue.BLUE);
        } else {
            setColor(LEDConstants.ColorValue.RED);
        }
    }

    public void setColor(LEDConstants.ColorValue colorValue) {
        telemetry.addData(LEDConstants.kSubsystemName + "Current Color Position", colorValue.getColorPosition());

        LED.setPosition(colorValue.getColorPosition());
    }

    public void setPosition(double position){
        LED.setPosition(position);
    }

}
