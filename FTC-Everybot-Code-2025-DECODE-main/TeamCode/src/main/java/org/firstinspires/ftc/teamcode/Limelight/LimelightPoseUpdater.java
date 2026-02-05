package org.firstinspires.ftc.teamcode.Limelight;

import com.pedropathing.geometry.Pose;

public class LimelightPoseUpdater {

    private static final double METERS_TO_INCHES = 39.3701;

    private final KalmanFilter xFilter;
    private final KalmanFilter yFilter;


    public LimelightPoseUpdater(Pose initialPose) {
        this.xFilter = new KalmanFilter(0.2, 0.5, initialPose.getX());
        this.yFilter = new KalmanFilter(0.2, 0.5, initialPose.getY());
    }


    public Pose getFusedPose(Pose limelightPose, double currentHeading) {
        double fusedX = xFilter.update(limelightPose.getX());
        double fusedY = yFilter.update(limelightPose.getY());

        return new Pose(fusedX, fusedY, currentHeading);
    }


    public static Pose convertLimelightToPedro(double limelightX,
                                               double limelightY,
                                               double currentHeading) {

        double xInches = limelightX * METERS_TO_INCHES;
        double yInches = limelightY * METERS_TO_INCHES;

        double pedroX = yInches + 72;
        double pedroY = -xInches + 72;

        return new Pose(pedroX, pedroY, currentHeading);
    }


    private static class KalmanFilter {
        private final double processNoise;
        private final double measurementNoise;

        private double estimate;
        private double errorCovariance = 1.0;

        public KalmanFilter(double processNoise,
                            double measurementNoise,
                            double initialEstimate) {
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
            this.estimate = initialEstimate;
        }


        public double update(double measurement) {
            errorCovariance += processNoise;

            double kalmanGain =
                    errorCovariance / (errorCovariance + measurementNoise);

            estimate = estimate + kalmanGain * (measurement - estimate);
            errorCovariance = (1.0 - kalmanGain) * errorCovariance;

            return estimate;
        }
    }
}
