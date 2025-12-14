package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

public class LEDConstants {
    public static final String kSubsystemName = "LED ";
    public static final String kLedServoID = "led";

    public enum ColorValue {
        OFF(kOffValue),
        RED(kRedValue),
        ORANGE(kOrangeValue),
        YELLOW(kYellowValue),
        SAGE(kSageValue),
        GREEN(kGreenValue),
        AZURE(kAzureValue),
        BLUE(kBlueValue),
        INDIGO(kIndigoVaue),
        VIOLET(kVioletValue),
        WHITE(kWhiteValue);

        private double colorPosition;

        private ColorValue(double colorPosition) {
            this.colorPosition = colorPosition;
        }

        public double getColorPosition() {
            return colorPosition;
        }
    }

    public static double testColor = 0.279;
    private static final double kOffValue = 0.000;
    private static final double kRedValue = 0.279;
    private static final double kOrangeValue = 0.333;
    private static final double kYellowValue = 0.388;
    private static final double kSageValue = 0.444;
    private static final double kGreenValue = 0.500;
    private static final double kAzureValue = 0.555;
    private static final double kBlueValue = 0.611;
    private static final double kIndigoVaue = 0.666;
    private static final double kVioletValue = 0.722;
    private static final double kWhiteValue = 1.0;
}
