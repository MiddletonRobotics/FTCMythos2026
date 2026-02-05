package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.vision.FiducialData3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;

import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.stream.Collectors;

public class Vision extends SubsystemBase {
    private final Limelight3A limelight;

    private LLResult llResult;

    private List<Integer> fidicualID = Collections.emptyList();
    private double tx = Double.POSITIVE_INFINITY;
    private double ty = Double.POSITIVE_INFINITY;
    private double ta = Double.POSITIVE_INFINITY;

    private VisionConstants.MotifPattern measuredPattern = VisionConstants.MotifPattern.NONE;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public Vision(HardwareMap hMap, TelemetryManager telemetryM) {
        limelight = hMap.get(Limelight3A.class, VisionConstants.limelightID);
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(20);
        limelight.start();

        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        llResult = limelight.getLatestResult();
        fidicualID = getAllVisibleFiducialIDs();

        telemetryM.addData(VisionConstants.kSubsystemName + " Fidicual Tx", tx);
        telemetryM.addData(VisionConstants.kSubsystemName + "Fidicual Ty", ty);
        telemetryM.addData(VisionConstants.kSubsystemName + "Fidicual Ta", ta);
        telemetryM.addData(VisionConstants.kSubsystemName + "Fidicual ID", fidicualID);
        telemetryM.addData(VisionConstants.kSubsystemName + "MOTIF Pattern", measuredPattern.toString());
    }

    public LLResult getLatestResult() {
        return llResult;
    }

    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        return getLatestResult().getFiducialResults();
    }

    public void readMotifPattern() {
        if (llResult == null) return;

        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return;

        for (LLResultTypes.FiducialResult fiducialResult : fiducials) {
            if (fiducialResult == null) continue;
            int id = fiducialResult.getFiducialId();

            if (id == 21 || id == 22 || id == 23) {
                setMotifPattern(VisionConstants.MotifPattern.fromFiducialId(id));
                return;
            }
        }
    }

    private boolean isTagPresentOnGoal(int id) {
        return id == 20 || id == 24;
    }

    private boolean isTagForAlliance(int id, GlobalConstants.AllianceColor alliance) {
        switch (alliance) {
            case BLUE:
                return id == 20;
            case RED:
                return id == 24;
            default:
                return false;
        }
    }

    public List<Integer> getAllVisibleFiducialIDs() {
        if (llResult == null) return Collections.emptyList();

        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return Collections.emptyList();
        }

        return fiducials.stream()
                .filter(Objects::nonNull)
                .map(LLResultTypes.FiducialResult::getFiducialId)
                .collect(Collectors.toList());
    }

    public Optional<FiducialData3D> getEstimatedRobotPose(GlobalConstants.AllianceColor alliance) {
        if (llResult == null) return Optional.empty();

        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            tx = Double.POSITIVE_INFINITY;
            ty = Double.POSITIVE_INFINITY;
            ta = Double.POSITIVE_INFINITY;

            return Optional.empty();
        }

        return fiducials.stream()
            .filter(Objects::nonNull)
            .filter(f -> isTagPresentOnGoal(f.getFiducialId()))
            .findFirst()
            .map(fiducial -> {
                tx = fiducial.getTargetXDegrees();
                ty = fiducial.getTargetYDegrees();
                ta = fiducial.getTargetArea();

                Pose3D robotFTCPose3D = fiducial.getRobotPoseFieldSpace();

                Pose convertedPose = new Pose(
                        robotFTCPose3D.getPosition().x,
                        robotFTCPose3D.getPosition().y,
                        robotFTCPose3D.getOrientation().getYaw(),
                        FTCCoordinates.INSTANCE
                ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                return new FiducialData3D(convertedPose, fiducial.getFiducialId());
            });
    }

    public boolean shouldTrustVision(FiducialData3D data, Pose currentRobotPose) {
        Pose visionPose = data.getRobotPose();

        double deltaX = Math.abs(visionPose.getX() - currentRobotPose.getX());
        double deltaY = Math.abs(visionPose.getY() - currentRobotPose.getY());

        if (deltaX > 2 || deltaY > 2) {
            return false;
        }

        // Check tag area (closer tags are more reliable)
        return !(llResult.getTa() < 0.1);
    }

    @Deprecated
    public static OptionalInt motifToNumber(VisionConstants.MotifPattern pattern) {
        if (pattern == null) return OptionalInt.empty();

        switch (pattern) {
            case GPP: return OptionalInt.of(1);
            case PGP: return OptionalInt.of(3);
            case PPG: return OptionalInt.of(5);
            default:  return OptionalInt.empty();
        }
    }

    public void setMotifPattern(VisionConstants.MotifPattern motifPattern) {
        this.measuredPattern = motifPattern;
    }
}