package org.firstinspires.ftc.library.settings;

import org.firstinspires.ftc.library.vision.ResolutionType;
import org.firstinspires.ftc.vision.VisionPortal;

public class CameraSettings {

    private final String key;
    private final ResolutionType resolutionType;
    private final double cameraFOV;
    private final CameraPosition cameraPosition;
    private final boolean enableLiveViewing;
    private final boolean showVisionStatistics;
    private final VisionPortal.StreamFormat streamFormat;

    public CameraSettings(
        final String key,
        final ResolutionType resolutionType,
        final double cameraFOV,
        final CameraPosition cameraPosition,
        final boolean enableLiveViewing,
        final boolean showVisionStatistics,
        final VisionPortal.StreamFormat streamFormat
    ) {
        this.key = key;
        this.resolutionType = resolutionType;
        this.cameraFOV = cameraFOV;
        this.cameraPosition = cameraPosition;
        this.enableLiveViewing = enableLiveViewing;
        this.showVisionStatistics = showVisionStatistics;
        this.streamFormat = streamFormat;
    }

    public String getName() {
        return key;
    }

    public ResolutionType getResolutionType() {
        return resolutionType;
    }

    public double getCameraFOV() {
        return cameraFOV;
    }

    public CameraPosition getPosition() {
        return cameraPosition;
    }

    public VisionPortal.StreamFormat getStreamFormat() {
        return streamFormat;
    }

    public boolean isLiveViewingEnabled() {
        return enableLiveViewing;
    }

    public boolean isShowVisionStatisticsEnabled() {
        return showVisionStatistics;
    }
}
