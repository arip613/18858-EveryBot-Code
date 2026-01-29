package org.firstinspires.ftc.teamcode.Limelight;

import com.pedropathing.geometry.Pose;

public class LimelightPoseUpdater {

    private static final double METERS_TO_INCHES = 39.3701;

    private final KalmanFilter xFilter;
    private final KalmanFilter yFilter;

    public LimelightPoseUpdater(Pose currentPose, Pose limelightPose) {
        this(currentPose, limelightPose, 0.2, 0.5);
    }

    public LimelightPoseUpdater(Pose currentPose, Pose limelightPose, double processNoise, double measurementNoise) {
        this.xFilter = new KalmanFilter(processNoise, measurementNoise);
        this.yFilter = new KalmanFilter(processNoise, measurementNoise);
    }

    public Pose getFusedPose(Pose currentPose, Pose limelightPose) {
        double fusedX = xFilter.update(limelightPose.getX(), currentPose.getX());
        double fusedY = yFilter.update(limelightPose.getY(), currentPose.getY());
        double fusedHeading = currentPose.getHeading();

        return new Pose(fusedX, fusedY, fusedHeading);
    }

    public static Pose convertLimelightToPedro(double limelightX, double limelightY, double heading) {
        double xInches = limelightX * METERS_TO_INCHES;
        double yInches = limelightY * METERS_TO_INCHES;

        double pedroX = yInches + 72;
        double pedroY = -xInches + 72;

        return new Pose(pedroX, pedroY, heading);
    }

    private static class KalmanFilter {
        private double processNoise;
        private double measurementNoise;
        private double estimate;
        private double errorCovariance;

        public KalmanFilter(double processNoise, double measurementNoise) {
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
            this.estimate = 0;
            this.errorCovariance = 1;
        }

        public double update(double measurement, double prediction) {
            errorCovariance += processNoise;
            double kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
            estimate = prediction + kalmanGain * (measurement - prediction);
            errorCovariance = (1 - kalmanGain) * errorCovariance;
            return estimate;
        }
    }
}