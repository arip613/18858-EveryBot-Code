package org.firstinspires.ftc.teamcode.Prism;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

// CLASS MADE BY JADEN
// Quote from ARI IN COMMITS:
// "will anyone even read this, sometimes i feel like im typing to nobody, actually ive decided this is my diary now"

/**
 * Handles proximity-based LED feedback for manual driving mode.
 * Shows discrete color steps based on distance to target (within 40 inches):
 * Red -> Orange -> Yellow -> Yellow-Green -> Light Green -> Full Green
 * Blinks when heading is aligned (within 10 degrees).
 */
public class HeatMapController {

    private static final double PROXIMITY_THRESHOLD = 40; // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(10);

    // Discrete color thresholds (distances from target)
    private static final double[] DISTANCE_THRESHOLDS = {33.3, 26.6, 20.0, 13.3, 6.6, 0.0};

    // RGB values for each color step
    private static final int[][] COLORS = {
            {255, 0, 0},      // Red
            {255, 165, 0},    // Orange
            {255, 255, 0},    // Yellow
            {154, 205, 50},   // Yellow-Green
            {144, 238, 144},  // Light Green
            {0, 255, 0}       // Full Green
    };

    // Blink settings
    private static final double BLINK_PERIOD_MS = 300;
    private static final double BLINK_ON_PERIOD_MS = 150;

    // ==================== STATE VARIABLES ====================
    private int currentR = 255;
    private int currentG = 0;
    private int currentB = 0;

    private boolean isActive = false;
    private boolean wasAlignedLastFrame = false;

    // ==================== HARDWARE ====================
    private GoBildaPrismDriver prism;
    private PrismAnimations.Solid proximitySolid;
    private PrismAnimations.Blink proximityBlink;

    // ==================== CONSTRUCTOR ====================
    public HeatMapController(GoBildaPrismDriver prism) {
        this.prism = prism;

        // Initialize solid animation for color display
        proximitySolid = new PrismAnimations.Solid(new Color(255, 0, 0));
        proximitySolid.setBrightness(100);

        // Initialize blink animation (will update colors dynamically)
        proximityBlink = new PrismAnimations.Blink(new Color(255, 0, 0), new Color(0, 0, 0));
        proximityBlink.setBrightness(100);
        proximityBlink.setPeriod((int) BLINK_PERIOD_MS);
        proximityBlink.setPrimaryColorPeriod((int) BLINK_ON_PERIOD_MS);
    }

    // ==================== PUBLIC METHODS ====================

    /**
     * Updates the heatmap LED display based on current position and target.
     * @param follower The robot's follower instance
     * @param targetPose The target pose to navigate to
     * @return true if heatmap is active (within threshold), false otherwise
     */
    public boolean update(Follower follower, Pose targetPose) {
        Pose currentPose = follower.getPose();

        // Calculate distance to target
        double dx = currentPose.getX() - targetPose.getX();
        double dy = currentPose.getY() - targetPose.getY();
        double distance = Math.hypot(dx, dy);

        if (distance > PROXIMITY_THRESHOLD) {
            isActive = false;
            wasAlignedLastFrame = false;
            return false;
        }

        isActive = true;

        // Calculate heading error
        double headingError = normalizeAngle(currentPose.getHeading() - targetPose.getHeading());
        boolean isAligned = Math.abs(headingError) < HEADING_TOLERANCE;

        // Determine color based on discrete distance thresholds
        int colorIndex = 0;
        for (int i = 0; i < DISTANCE_THRESHOLDS.length; i++) {
            if (distance >= DISTANCE_THRESHOLDS[i]) {
                colorIndex = i;
                break;
            }
        }

        // Set current color to the discrete step
        currentR = COLORS[colorIndex][0];
        currentG = COLORS[colorIndex][1];
        currentB = COLORS[colorIndex][2];

        // Update LED display based on alignment
        boolean alignmentChanged = isAligned != wasAlignedLastFrame;

        if (isAligned) {
            // Blink the current proximity color
            proximityBlink.setPrimaryColor(new Color(currentR, currentG, currentB));
            proximityBlink.setSecondaryColor(new Color(0, 0, 0));

            // Only re-insert when switching from solid to blink
            if (alignmentChanged) {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, proximityBlink);
            } else {
                // Update existing animation
                prism.updateAnimationFromIndex(LayerHeight.LAYER_0);
            }
        } else {
            // Solid proximity color
            proximitySolid.setPrimaryColor(new Color(currentR, currentG, currentB));

            // Always update when in solid mode or when transitioning
            if (alignmentChanged || !wasAlignedLastFrame) {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, proximitySolid);
            } else {
                prism.updateAnimationFromIndex(LayerHeight.LAYER_0);
            }
        }

        wasAlignedLastFrame = isAligned;
        return true;
    }

    /**
     * @return true if the heatmap is currently active (robot is within proximity threshold)
     */
    public boolean isActive() {
        return isActive;
    }

    /**
     * @return Current red color value (0-255)
     */
    public int getCurrentR() {
        return currentR;
    }

    /**
     * @return Current green color value (0-255)
     */
    public int getCurrentG() {
        return currentG;
    }

    /**
     * @return Current blue color value (0-255)
     */
    public int getCurrentB() {
        return currentB;
    }

    /**
     * Resets the heatmap controller to its initial state
     */
    public void reset() {
        isActive = false;
        wasAlignedLastFrame = false;
        currentR = 255;
        currentG = 0;
        currentB = 0;
    }

    /**
     * Normalizes an angle to the range [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}