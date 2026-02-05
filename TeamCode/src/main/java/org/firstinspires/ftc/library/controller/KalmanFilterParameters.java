package org.firstinspires.ftc.library.controller;

public class KalmanFilterParameters {
    public double modelCovariance;
    public double dataCovariance;

    /**
     * This creates a new KalmanFilterParameters with a specified model and data covariance.
     *
     * @param modelCovariance the covariance of the model.
     * @param dataCovariance the covariance of the data.
     */
    public KalmanFilterParameters(double modelCovariance, double dataCovariance) {
        this.modelCovariance = modelCovariance;
        this.dataCovariance = dataCovariance;
    }
}