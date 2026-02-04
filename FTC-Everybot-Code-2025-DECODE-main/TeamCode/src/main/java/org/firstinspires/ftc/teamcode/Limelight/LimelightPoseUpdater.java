package org.firstinspires.ftc.teamcode.Limelight;

import com.pedropathing.geometry.Pose;

public class LimelightPoseUpdater {

    private static final double METERS_TO_INCHES = 39.3701;

    private final KalmanFilter xFilter;
    private final KalmanFilter yFilter;

    /**
     * Create a pose updater seeded with a known-good pose
     * (this should be your first unfused Limelight pose).
     */
    public LimelightPoseUpdater(Pose initialPose) {
        this.xFilter = new KalmanFilter(0.2, 0.5, initialPose.getX());
        this.yFilter = new KalmanFilter(0.2, 0.5, initialPose.getY());
    }

    /**
     * Fuse a Limelight pose into the current estimate.
     * Heading is intentionally NOT fused.
     */
    public Pose getFusedPose(Pose limelightPose, double currentHeading) {
        double fusedX = xFilter.update(limelightPose.getX());
        double fusedY = yFilter.update(limelightPose.getY());

        return new Pose(fusedX, fusedY, currentHeading);
    }

    /**
     * Convert Limelight meters into Pedro field coordinates (inches).
     */
    public static Pose convertLimelightToPedro(double limelightX,
                                               double limelightY,
                                               double currentHeading) {

        double xInches = limelightX * METERS_TO_INCHES;
        double yInches = limelightY * METERS_TO_INCHES;

        // Field transform (matches your existing logic)
        double pedroX = yInches + 72;
        double pedroY = -xInches + 72;

        return new Pose(pedroX, pedroY, currentHeading);
    }

    /**
     * Simple 1D Kalman filter for FTC-scale localization correction.
     */
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
