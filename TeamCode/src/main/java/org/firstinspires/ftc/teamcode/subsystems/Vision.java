package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.vision.FiducialData3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalInt;

public class Vision extends SubsystemBase {
    private final Limelight3A limelight;

    private LLResult llResult;
    private int fidicualID;
    private double tx;
    private double ty;
    private double ta;
    private double distance;

    private VisionConstants.MotifPattern measuredPattern = VisionConstants.MotifPattern.NONE;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public Vision(HardwareMap hMap, TelemetryManager telemetryM) {
        limelight = hMap.get(Limelight3A.class, VisionConstants.limelightID);
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);
        limelight.start();

        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        llResult = limelight.getLatestResult();
        Optional<FiducialData3D> data = getAllianceTagInfo(GlobalConstants.allianceColor);
        data.ifPresent(fiducialData3D -> telemetryM.addData("Vision Distance", fiducialData3D.distanceMeters));

        telemetryM.addData("Vision Tx", tx);
        telemetryM.addData("Vision Ty", ty);
        telemetryM.addData("Vision Ta", ta);
        telemetryM.addData("Vision RLD", getFidicualDistance(30, 317.5, 749.3));
        telemetryM.addData("Current Loaded Pattern", measuredPattern.toString());
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

    public Optional<FiducialData3D> getAllianceTagInfo(GlobalConstants.AllianceColor alliance) {
        if (llResult == null) return Optional.empty();

        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            fidicualID = -1;
            tx = -1;
            ty = -1;
            ta = -1;

            return Optional.empty();
        }

        return fiducials.stream()
            .filter(Objects::nonNull)
            .filter(f -> isTagForAlliance(f.getFiducialId(), alliance))
            .findFirst()
            .map(fiducial -> {
                fidicualID = fiducial.getFiducialId();
                tx = fiducial.getTargetXDegrees();
                ty = fiducial.getTargetYDegrees();
                ta = fiducial.getTargetArea();

                Pose3D robotFTCPose3D = fiducial.getRobotPoseFieldSpace();
                Pose2D robotFTCPose2D = new Pose2D(
                    DistanceUnit.INCH,
                    robotFTCPose3D.getPosition().x,
                    robotFTCPose3D.getPosition().y,
                    AngleUnit.RADIANS,
                    robotFTCPose3D.getOrientation().getYaw(AngleUnit.RADIANS)
                );

                Pose convertedPose = PoseConverter.pose2DToPose(robotFTCPose2D, InvertedFTCCoordinates.INSTANCE);
                distance = Math.sqrt(
                        robotFTCPose3D.getPosition().x * robotFTCPose3D.getPosition().x +
                        robotFTCPose3D.getPosition().y * robotFTCPose3D.getPosition().y +
                        robotFTCPose3D.getPosition().z * robotFTCPose3D.getPosition().z
                );

                return new FiducialData3D(convertedPose, distance, fiducial.getFiducialId());
            });
    }

    public double getFidicualDistance(double limelightAngle, double limelightHeight, double aprilTagHeight) {
        double angleToGoalDegrees = limelightAngle + limelight.getLatestResult().getTy();
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        return (aprilTagHeight - limelightHeight) / Math.tan(angleToGoalRadians);
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
