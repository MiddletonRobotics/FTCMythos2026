package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.library.math.geometry.Transform2d;
import org.firstinspires.ftc.library.math.geometry.Units;

@Configurable
public class VisionConstants {
    public static final String kSubsystemName = "Vision ";
    public static final String limelightID = "limelight";
    public static final double limelightHeight = 12.5; //inches

    public enum MotifPattern {
        NONE(-1),
        PPG(21),
        PGP(22),
        GPP(23);

        private int fidicualID;
        private MotifPattern(int fidicualID) {
            this.fidicualID = fidicualID;
        }

        public int getFidicualID() {
            return fidicualID;
        }

        public static MotifPattern fromFiducialId(int id) {
            for (MotifPattern pattern : values()) {
                if (pattern.fidicualID == id) return pattern;
            }

            return null;
        }
    }
}
