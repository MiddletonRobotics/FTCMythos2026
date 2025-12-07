package org.firstinspires.ftc.library.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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

    public double getFidicualDistance(double[] cameraSpacePose) {
        Pose3D cameraSpace = limelight.getLatestResult().getFiducialResults().get(0).getTargetPoseCameraSpace();

        double x =  cameraSpace.getPosition().x;
        double y =  cameraSpace.getPosition().y;
        double z =  cameraSpace.getPosition().z;

        return Math.sqrt(x*x + y*y + z*z);
    }

    public double getFidicualDistance(double limelightAngle, double limelightHeight, double aprilTagHeight) {
        double angleToGoalDegrees = limelightAngle + limelight.getLatestResult().getTy();
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        return (aprilTagHeight - limelightHeight) / Math.tan(angleToGoalRadians);
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
