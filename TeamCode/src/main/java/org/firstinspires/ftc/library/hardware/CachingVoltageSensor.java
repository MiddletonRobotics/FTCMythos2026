package org.firstinspires.ftc.library.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CachingVoltageSensor {

    public static int voltageIndex = 0;
    public static boolean cached = false;

    public static double nominalVoltage = 12.5; // This is the voltage we want the robot to always try to operate at.
    public static double cacheInvalidateSeconds = 0.5;

    private VoltageSensor voltageSensor;
    public double voltage = 0;
    private ElapsedTime timer = new ElapsedTime();

    public CachingVoltageSensor(HardwareMap hmap) {
        voltageSensor = hmap.getAll(VoltageSensor.class).get(voltageIndex); // Voltages of both hubs should be roughly equal to each other
        timer.reset();
    }

    /**
     * @return The last cached voltage measurement.
     */

    public double getVoltage() {
        if (timer.seconds() > cacheInvalidateSeconds && cacheInvalidateSeconds >= 0) {
            clearCache();
        }

        if (!cached) {
            cached = true;
            return voltage = voltageSensor.getVoltage();
        } else {
            return voltage;
        }
    }

    /**
     * @return A scalar that normalizes power outputs to the nominal voltage from the current voltage.
     */

    public double getVoltageNormalized() {
        return nominalVoltage / getVoltage();
    }

    /**
     * Forcibly invalidates the cache.
     */

    public void clearCache() {
        cached = false;
    }

}