package org.firstinspires.ftc.library.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.Queue;

/**
 * Represents an analog ultrasonic distance sensor connected to an analog input port.
 *
 * <p>This class reads raw voltages from the sensor, maintains a rolling average of recent
 * readings to reduce noise, and converts the result to a {@link DistanceUnit}-safe value.</p>
 *
 * <p>Typical usage: Call {@link #calculateAverage()} periodically (e.g. in {@code periodic()})
 * to update the rolling average, then read the smoothed distance from {@link #getDistance()}.</p>
 *
 * <p><b>Note:</b> The conversion factor used here assumes the sensor outputs approximately
 * {@code 3.3V = 500 cm} (or about {@code 0.0066 V/cm}). Adjust this constant for your sensor.</p>
 */

public class AnalogUltrasonic {

    /** The Analog Input used to read data from the sensor */
    private final AnalogInput device;

    /** The default unit to return (default is in millimeters)  */
    private DistanceUnit unit = DistanceUnit.MM;

    /** The number of recent samples to average together. */
    public int rollingAverageSize = 3;

    /** The most recent rolling average of the voltage samples. */
    private double deviceAverage;

    /** A queue of recent voltage readings for rolling average smoothing. */
    private final Queue<Double> deviceData = new LinkedList<>();

    /**
     * Creates a new {@code AnalogUltrasonic} sensor object.
     *
     * @param hMap The active {@link HardwareMap} to get the analog input from.
     * @param deviceName The configured name of the analog sensor in the hardware map.
     */

    public AnalogUltrasonic(HardwareMap hMap, String deviceName) {
        this.device = hMap.get(AnalogInput.class, deviceName);
    }

    /**
     * Updates the rolling average of recent voltage samples.
     *
     * <p>Call this method periodically to keep the smoothed distance reading fresh.</p>
     */

    public void calculateAverage() {
        deviceData.add(device.getVoltage());
        if (deviceData.size() > rollingAverageSize) {
            deviceData.remove();
        }

        deviceAverage = deviceData.stream().reduce((total, el) -> total + el / deviceData.size()).get();
    }

    /**
     * Sets the distance unit to use when returning distances.
     *
     * @param unit The desired {@link DistanceUnit} (e.g. INCH, CM, MM).
     */

    public void setDistanceUnit(DistanceUnit unit) {
        this.unit = unit;
    }

    /**
     * Gets the smoothed distance reading from the sensor.
     *
     * <p>Converts the average voltage to distance using a fixed conversion factor,
     * assuming 500cm is the maximum voltage (3.3V).</p>
     *
     * @return The distance in the configured {@link #unit}.
     */

    public double getDistance() {
        return unit.fromCm(deviceAverage * 500 / 3.3);
    }
}