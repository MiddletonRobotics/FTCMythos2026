package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED extends SubsystemBase {
    private Servo LED;

    private Telemetry telemetry;

    private static LED instance;
    public static LED getInstance(HardwareMap hMap, Telemetry telemetry) {
        if(instance == null) {
            instance = new LED(hMap, telemetry);
        }

        return instance;
    }

    private LED(HardwareMap hMap, Telemetry telemetry) {
        LED = hMap.get(Servo.class, "led");
        this.telemetry = telemetry;
    }

    public void setColor(double color) {
        telemetry.addData("LEDCurrentColor", color);
        LED.setPosition(color);
    }
}
