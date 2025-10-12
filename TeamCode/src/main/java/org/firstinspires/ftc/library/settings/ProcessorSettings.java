package org.firstinspires.ftc.library.settings;

public class ProcessorSettings {
    private boolean do3DTracking;
    private double[] lensIntrinsics;

    public ProcessorSettings(final boolean do3DTracking, final double[] lensIntrinsics) {
        this.do3DTracking = do3DTracking;
        this.lensIntrinsics = lensIntrinsics;
    }

    public boolean is3DTracking() {
        return do3DTracking;
    }

    public double[] getResolution() {
        return lensIntrinsics;
    }
}
