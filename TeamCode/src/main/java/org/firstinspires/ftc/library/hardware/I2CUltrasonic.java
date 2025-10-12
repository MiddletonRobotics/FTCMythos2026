package org.firstinspires.ftc.library.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class I2CUltrasonic {
    private final I2cDeviceSynch device;

    private static final int I2C_ADDRESS = 0x11;
    private static final int CMD_REGISTER = 0x01;
    private static final int DIST_HIGH = 0x02;
    private static final int DIST_LOW = 0x03;

    private boolean waitingForResult = false;
    private long measurementStartTime = 0;
    private int lastDistance = -1;

    public I2CUltrasonic(HardwareMap hMap, String name) {
        I2cDeviceSynch deviceSync = hMap.get(I2cDeviceSynch.class, name);
        this.device = deviceSync;
        this.device.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));
        this.device.engage();
    }

    public void update() {
        long currentTime = System.currentTimeMillis();

        if (!waitingForResult) {
            // Start a new measurement
            device.write8(CMD_REGISTER, 0x01);
            measurementStartTime = currentTime;
            waitingForResult = true;
        } else if (currentTime - measurementStartTime >= 60) {
            // Time has passed, read result
            byte high = device.read8(DIST_HIGH);
            byte low = device.read8(DIST_LOW);
            lastDistance = ((high & 0xFF) << 8) | (low & 0xFF);
            waitingForResult = false;
        }
    }

    public int getLastDistance() {
        return lastDistance;
    }

    public boolean hasNewReading() {
        return !waitingForResult;
    }
}