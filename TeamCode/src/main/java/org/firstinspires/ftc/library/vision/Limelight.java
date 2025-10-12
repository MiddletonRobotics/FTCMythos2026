package org.firstinspires.ftc.library.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight {
    private final Limelight3A limelight;

    public enum LimelightMode {
        FIDICUAL,
        COLOR_THRESHOLDING,
        NEURAL_DETECTOR,
        PYTHON_SNAPSCRIPT
    }

    private LimelightMode limelightMode;

    private LLResult result;
    private List<LLResultTypes.FiducialResult> fiducialResults;
    private List<LLResultTypes.ColorResult> colorResults;
    private List<LLResultTypes.DetectorResult> detectorResults;

    public Limelight(final HardwareMap hMap, final String name, LimelightMode limelightMode) {
        limelight = hMap.get(Limelight3A.class, name);
        this.limelightMode = limelightMode;
    }

    public boolean checkForResult() {
        LLResult result = limelight.getLatestResult();
        return result.isValid();
    }

    /*

    public double getTx() {
        switch (limelightMode) {
            case FIDICUAL:
                if(checkForResult())
        }
    }

     */
}
