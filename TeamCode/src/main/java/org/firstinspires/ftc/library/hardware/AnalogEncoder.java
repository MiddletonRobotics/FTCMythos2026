package org.firstinspires.ftc.library.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Represents an analog absolute encoder connected to an analog input port.
 *
 * <p>This class reads the encoder's voltage and converts it into an absolute angle.
 * It assumes the sensor outputs a voltage proportional to the shaft's absolute rotation.</p>
 *
 * <p>Typical usage includes setting the maximum output voltage (commonly 3.3 V or 5 V),
 * configuring the maximum angle (usually 360° per revolution), applying an offset for
 * zeroing, and optionally inverting the reading.</p>
 *
 * <p><b>Note:</b> On the REV Control Hub, the analog input voltage range is 0–3.3 V.
 * Most analog encoders should be powered from the 3.3 V rail.</p>
 */

public class AnalogEncoder {

    /** The Analog Input used to read data from the sensor */
    private AnalogInput sensor;

    /** Position offset for the encoder, in degrees */
    private double offset;

    /** The maximum voltage written when the sensor it at its maximum position */
    private double maximumVoltage;

    /** Whether to invert the direction of the encoder */
    private boolean inverted;

    /** The maximum angle corresponding to {@link #maximumVoltage}, usually 360°. */
    private double maximumAngle;

    {
        maximumVoltage = 3.3; // default to 3.3V
        maximumAngle = 360; // default to 360 degrees
        offset = 0; // default to 0 offset
        inverted = false; // default to not inverted
    }

    /**
     * Creates a new {@code AnalogEncoder} using the specified hardware map name.
     *
     * @param hMap  The active {@link HardwareMap} to get the device from.
     * @param deviceName   The name of the analog input device in the hardware map.
     */

    public AnalogEncoder(HardwareMap hMap, String deviceName){
        this.sensor = hMap.get(AnalogInput.class, deviceName);
    }

    /**
     * Sets whether the encoder readings should be inverted.
     * @param inverted {@code true} to invert the reading direction, {@code false} otherwise.
     */

    public void setInverted(boolean inverted){
        this.inverted = inverted;
    }

    /**
     * Sets the angle offset of the encoder. This offset is added to the measured angle and can be
     * used to "zero" the encoder.
     *
     * @param offset The offset in degrees.
     */

    public void setPositionOffset(double offset){
        this.offset = offset;
    }

    /**
     * Sets the expected maximum output voltage from the encoder.
     * @param maximumVoltage The voltage corresponding to the encoder's maximum angle (e.g. 3.3 or 5.0).
     */

    public void setMaximumVoltage(double maximumVoltage){
        this.maximumVoltage = maximumVoltage;
    }

    /**
     * Sets the maximum angle of the analog encoder. This means that 3.3V = max angle.
     * @param maximumAngle Angle at 3.3V.
     */
    public void setMaximumAngle(double maximumAngle) {
        this.maximumAngle = maximumAngle;
    }

    /**
     * Gets the current angle from the encoder.
     * @return The absolute angle as a {@link Rotation2d}.
     */

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(AngleUnit.normalizeDegrees((inverted ? -1 : 1) * (sensor.getVoltage() * maximumAngle / maximumVoltage) + offset));
    }
}