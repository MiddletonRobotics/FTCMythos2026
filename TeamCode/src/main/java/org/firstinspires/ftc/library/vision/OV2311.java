package org.firstinspires.ftc.library.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.settings.CameraSettings;
import org.firstinspires.ftc.library.settings.ProcessorSettings;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.EnumSet;
import java.util.List;

public class OV2311 {

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    EnumSet<ResolutionType> allowedResolutions = EnumSet.of(
            ResolutionType.k1600x1300,
            ResolutionType.k1280x720,
            ResolutionType.k640x480
    );

     private final String invalidResolution = "The OV2311 does not support the resolution loaded onto the camera currently. The only supported resolutions" +
            " are 1600x1300, 1280x720 and 640x480. Please use one of the previously listed resolutions instead of ";

    private final boolean is3DTracking;
    private final double cameraFOV;

    /**
     * Creates and starts an AprilTag vision system using the specified webcam name.
     *
     * @param hMap hardwareMap from OpMode
     * @param cameraSettings camera settings to pull data from
     */

    public OV2311(HardwareMap hMap, ProcessorSettings processorSettings, CameraSettings cameraSettings) {
        this.is3DTracking = processorSettings.is3DTracking();
        this.cameraFOV = cameraSettings.getCameraFOV();

        if(!allowedResolutions.contains(cameraSettings.getResolutionType())) {
            throw new IllegalArgumentException(invalidResolution + cameraSettings.getResolutionType().toString());
        }

        aprilTagProcessor = new AprilTagProcessor.Builder()
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(is3DTracking)
            .setDrawCubeProjection(is3DTracking)
            .setCameraPose(
                    new Position(
                        DistanceUnit.INCH,
                        cameraSettings.getPosition().getX(DistanceUnit.INCH),
                        cameraSettings.getPosition().getY(DistanceUnit.INCH),
                        cameraSettings.getPosition().getZ(DistanceUnit.INCH),
                        0
                    ), new YawPitchRollAngles(
                    AngleUnit.DEGREES,
                        cameraSettings.getPosition().getYaw(AngleUnit.DEGREES),
                        cameraSettings.getPosition().getPitch(AngleUnit.DEGREES),
                        cameraSettings.getPosition().getRoll(AngleUnit.DEGREES),
                        0
                    )
            ).build();

        visionPortal = new VisionPortal.Builder()
            .setCamera(hMap.get(WebcamName.class, cameraSettings.getName()))
            .setCameraResolution(cameraSettings.getResolutionType().getResolution())
            .setStreamFormat(cameraSettings.getStreamFormat())
            .addProcessor(aprilTagProcessor)
            .enableLiveView(cameraSettings.isLiveViewingEnabled())
            .setShowStatsOverlay(cameraSettings.isShowVisionStatisticsEnabled())
            .setAutoStopLiveView(true)
            .build();
    }

    private List<AprilTagDetection> getFreshDetections() {
        return aprilTagProcessor.getFreshDetections();
    }

    public int[] getFidicualIDs() {
        List<AprilTagDetection> detections = getFreshDetections();
        return detections.stream().mapToInt(detection -> detection.id).toArray();
    }

    public double[] getFidicualOffsets() { // TODO: Convert this function from meters to inches.
        List<AprilTagDetection> detections = getFreshDetections();
        if (detections.isEmpty()) return null;

        // TODO: Later sort by confidence, distance, or ambiguity
        AprilTagDetection detection = detections.get(0);

        if (detection.ftcPose == null) {
            // No valid 3D pose available
            return null;
        }

        // FTC Pose: x = left/right, y = up/down, z = forward/back
        // Already in meters according to FTC SDK docs
        double x = detection.ftcPose.x;
        double y = detection.ftcPose.y;
        double z = detection.ftcPose.z;

        return new double[] {x, y, z};
    }

    public double getFiducialAmbiguity() {
        List<AprilTagDetection> detections = getFreshDetections();
        if (detections.isEmpty() || !is3DTracking) return -1;

        AprilTagDetection detection = detections.get(0); // TODO: I don't know how ill sort this but similar to the getFidicualOffsets

        // Get the 4 tag corners
        // corners[0..3] are normalized (0-1) points in image space
        double[][] pts = new double[4][2];
        for (int i = 0; i < 4; i++) {
            pts[i][0] = detection.corners[i].x;
            pts[i][1] = detection.corners[i].y;
        }

        // Compute side lengths
        double[] sides = new double[4];
        for (int i = 0; i < 4; i++) {
            double dx = pts[(i + 1) % 4][0] - pts[i][0];
            double dy = pts[(i + 1) % 4][1] - pts[i][1];
            sides[i] = Math.hypot(dx, dy);
        }

        // Ratio between shortest and longest side (1 = perfect square, <1 = skewed)
        double minSide = Math.min(Math.min(sides[0], sides[1]), Math.min(sides[2], sides[3]));
        double maxSide = Math.max(Math.max(sides[0], sides[1]), Math.max(sides[2], sides[3]));
        double sideRatio = minSide / maxSide;

        // Optional: also check diagonals (more robust)
        double diag1 = Math.hypot(pts[2][0] - pts[0][0], pts[2][1] - pts[0][1]);
        double diag2 = Math.hypot(pts[3][0] - pts[1][0], pts[3][1] - pts[1][1]);
        double diagRatio = Math.min(diag1, diag2) / Math.max(diag1, diag2);

        // Combine into an ambiguity metric
        return (sideRatio + diagRatio) / 2.0;
    }

    /**
     * Returns the camera's pose in field coordinates, using
     * the first detected AprilTag and its known field position.
     *
     * @return Pose3D of the camera in field space, or null if unavailable
     */

    /*
    public Pose3D getCameraFieldPose() { // TODO: Need to fix this
        List<AprilTagDetection> detections = getFreshDetections();
        if (detections.isEmpty() || !is3DTracking) return null;

        AprilTagDetection detection = detections.get(0);

        if (detection.ftcPose == null || detection.metadata == null) {
            return null;
        }

        // Tag position in the field (from metadata) – note: only translation, no orientation
        VectorF tagFieldPos = detection.metadata.fieldPosition;

        // Camera pose relative to the tag
        double camXRel = detection.ftcPose.x;
        double camYRel = detection.ftcPose.y;
        double camZRel = detection.ftcPose.z;

        // Approximate camera position in field coords by offsetting from tag position
        double camXField = tagFieldPos.get(0) + camXRel;
        double camYField = tagFieldPos.get(1) + camYRel;
        double camZField = tagFieldPos.get(2) + camZRel;

        // FTC Pose gives us an orientation as well (Quaternion)
        Quaternion camOrientation = detection.ftcPose.getOrientation();

        // Build Pose3D in field space
        return new Pose3D(new Position((float) camXField, (float) camYField, (float) camZField), camOrientation
        );
    }
     */

    /**
     * Returns the straight-line  distance in inches from the camera to the center of the most confidently detected tag.
     * @return distance in inches, or -1 if no tag detected
     */

    public double distanceToTag() {
        List<AprilTagDetection> detections = getFreshDetections();
        if (detections.isEmpty() || !is3DTracking) return -1;

        AprilTagDetection detection = detections.get(0);

        double x = detection.ftcPose.x;
        double y = detection.ftcPose.y;
        double z = detection.ftcPose.z;

        return Math.sqrt(x * x + y * y + z * z);
    }

    public void disableLiveView() {
        visionPortal.stopLiveView();
    }

    public void enableLiveView() {
        visionPortal.resumeLiveView();
    }
}
