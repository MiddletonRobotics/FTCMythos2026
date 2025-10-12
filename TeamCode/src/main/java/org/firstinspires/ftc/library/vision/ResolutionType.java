package org.firstinspires.ftc.library.vision;

import android.util.Size;

public enum ResolutionType {
    k1600x1300(new Size(1600, 1200)),
    k1280x720(new Size(1280, 720)),
    k640x480(new Size(640, 480));

    private Size resolution;

    private ResolutionType(final Size resolution) {
        this.resolution = resolution;
    }

    public Size getResolution() {
        return resolution;
    }
}