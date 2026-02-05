package org.firstinspires.ftc.teamcode.Prism;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

/**
 * Handles proximity-based LED feedback for manual driving mode.
 * Shows red-to-green gradient based on distance to target (within 20 inches),
 * and blinks when heading is aligned (within 4 degrees).
 */
public class HeatMapController {

    private static final double PROXIMITY_THRESHOLD = 40; // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(10);
    private static final int COLOR_STEP = 15;

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

        // Initialize solid animation for gradient display
        proximitySolid = new PrismAnimations.Solid(Color.RED);
        proximitySolid.setBrightness(100);

        // Initialize blink animation (will update colors dynamically)
        proximityBlink = new PrismAnimations.Blink(Color.RED, new Color(0, 0, 0));
        proximityBlink.setBrightness(100);
        proximityBlink.setPeriod((int) BLINK_PERIOD_MS);
        proximityBlink.setPrimaryColorPeriod((int) BLINK_ON_PERIOD_MS);
    }

    // ==================== PUBLIC METHODS ====================


    public boolean update(Follower follower, Pose targetPose) {
        Pose currentPose = follower.getPose();

        // Calculate distance to target
        double dx = currentPose.getX() - targetPose.getX();
        double dy = currentPose.getY() - targetPose.getY();
        double distance = Math.hypot(dx, dy);

        // Only activate within proximity threshold
        if (distance > PROXIMITY_THRESHOLD) {
            isActive = false;
            wasAlignedLastFrame = false;
            return false;
        }

        isActive = true;

        // Calculate heading error
        double headingError = normalizeAngle(currentPose.getHeading() - targetPose.getHeading());
        boolean isAligned = Math.abs(headingError) < HEADING_TOLERANCE;

        // Calculate target color based on distance (red at 20", green at 0")
        double proximityRatio = 1.0 - (distance / PROXIMITY_THRESHOLD);
        proximityRatio = Math.max(0.0, Math.min(1.0, proximityRatio)); // Clamp to [0, 1]

        int targetR = (int) (255 * (1.0 - proximityRatio));
        int targetG = (int) (255 * proximityRatio);
        int targetB = 0;

        // Smoothly transition current color to target color
        currentR = stepToward(currentR, targetR);
        currentG = stepToward(currentG, targetG);
        currentB = stepToward(currentB, targetB);

        // Update LED display based on alignment
        boolean alignmentChanged = isAligned != wasAlignedLastFrame;

        if (isAligned) {
            // Blink the current proximity color
            // Always update color
            proximityBlink.setPrimaryColor(currentR, currentG, currentB);
            proximityBlink.setSecondaryColor(0, 0, 0);

            // Only re-insert when switching from solid to blink
            if (alignmentChanged) {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, proximityBlink);
            }
        } else {
            // Solid proximity color
            // Always update color and insert/update every frame for smooth gradient
            proximitySolid.setPrimaryColor(currentR, currentG, currentB);
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, proximitySolid);
        }

        wasAlignedLastFrame = isAligned;
        return true;
    }


    public boolean isActive() {
        return isActive;
    }


    public int getCurrentR() { return currentR; }
    public int getCurrentG() { return currentG; }
    public int getCurrentB() { return currentB; }


    public void reset() {
        isActive = false;
        wasAlignedLastFrame = false;
        currentR = 255;
        currentG = 0;
        currentB = 0;
    }


    private int stepToward(int current, int target) {
        if (current < target) {
            return Math.min(current + COLOR_STEP, target);
        } else if (current > target) {
            return Math.max(current - COLOR_STEP, target);
        }
        return current;
    }


    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}