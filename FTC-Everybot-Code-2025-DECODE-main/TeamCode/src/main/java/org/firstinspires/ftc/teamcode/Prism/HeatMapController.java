package org.firstinspires.ftc.teamcode.Prism;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

// CLASS MADE BY JADEN
// Quote from ARI IN COMMITS:
// "will anyone even read this, sometimes i feel like im typing to nobody, actually ive decided this is my diary now"

/**
 * Shot-confidence LED feedback driven by a table of "known good" shot poses.
 * Each update() picks whichever shot pose the robot is closest to and drives
 * the LEDs off that pose. Inside PROXIMITY_THRESHOLD the color fades smoothly
 * from red (edge of range) to green (at the setpoint). The LEDs pulse only
 * when BOTH the position AND heading are inside their tight tolerances.
 */
public class HeatMapController {

    private static final double PROXIMITY_THRESHOLD = 8.0; // inches — gradient range
    private static final double POSITION_TOLERANCE = 3.0;   // inches — needed for pulse
    private static final double HEADING_TOLERANCE = Math.toRadians(12);

    private static final int PULSE_PERIOD_MS = 250;
    private static final int PULSE_ON_PERIOD_MS = 125;

    private final GoBildaPrismDriver prism;
    private final PrismAnimations.Solid solid;
    private final PrismAnimations.Blink pulse;

    private final List<Pose> shotPoses = new ArrayList<>();

    private int currentR = 255;
    private int currentG = 0;
    private int currentB = 0;

    private boolean isActive = false;
    private boolean wasActive = false;
    private boolean wasPulsing = false;

    // Snapshot from the latest update() — for telemetry.
    private Pose lastPose = null;
    private Pose lastClosest = null;
    private int lastClosestIndex = -1;
    private double lastDistance = Double.NaN;
    private double lastHeadingErrorDeg = Double.NaN;
    private boolean lastPositionOk = false;
    private boolean lastHeadingOk = false;
    private int lastSegmentA = -1;
    private int lastSegmentB = -1;
    private double lastSegmentT = Double.NaN;

    public HeatMapController(GoBildaPrismDriver prism) {
        this.prism = prism;

        solid = new PrismAnimations.Solid(new Color(255, 0, 0));
        solid.setBrightness(100);

        pulse = new PrismAnimations.Blink(new Color(255, 0, 0), new Color(0, 0, 0));
        pulse.setBrightness(100);
        pulse.setPeriod(PULSE_PERIOD_MS);
        pulse.setPrimaryColorPeriod(PULSE_ON_PERIOD_MS);
    }

    // ==================== SHOT POSE TABLE ====================

    public void addShotPose(Pose pose) {
        shotPoses.add(pose);
    }

    public void addShotPose(double x, double y, double headingRadians) {
        shotPoses.add(new Pose(x, y, headingRadians));
    }

    public void clearShotPoses() {
        shotPoses.clear();
    }

    public List<Pose> getShotPoses() {
        return shotPoses;
    }

    // ==================== UPDATE ====================

    /**
     * Interpolates between shot poses: projects the robot onto the nearest segment
     * between any two registered poses and drives the LEDs off that projection.
     * Heading lerps linearly along the segment. With one registered pose, falls
     * back to point distance.
     * @return true while the heatmap owns the LED layer; false when out of range or empty.
     */
    public boolean update(Follower follower) {
        Pose currentPose = follower.getPose();
        lastPose = currentPose;

        if (shotPoses.isEmpty()) {
            deactivate();
            lastClosest = null;
            lastClosestIndex = -1;
            lastDistance = Double.NaN;
            lastHeadingErrorDeg = Double.NaN;
            lastSegmentA = -1;
            lastSegmentB = -1;
            lastSegmentT = Double.NaN;
            return false;
        }

        double rx = currentPose.getX();
        double ry = currentPose.getY();

        double effX, effY, effHeading, effDistance;
        int nearestEndpoint;
        int segA = -1, segB = -1;
        double segT = Double.NaN;

        if (shotPoses.size() == 1) {
            Pose p = shotPoses.get(0);
            effX = p.getX();
            effY = p.getY();
            effHeading = p.getHeading();
            effDistance = Math.hypot(rx - effX, ry - effY);
            nearestEndpoint = 0;
        } else {
            double bestDist = Double.MAX_VALUE;
            double bestEx = 0, bestEy = 0, bestEh = 0, bestT = 0;
            int bestA = 0, bestB = 1;
            for (int i = 0; i < shotPoses.size(); i++) {
                for (int j = i + 1; j < shotPoses.size(); j++) {
                    Pose A = shotPoses.get(i);
                    Pose B = shotPoses.get(j);
                    double ax = A.getX(), ay = A.getY();
                    double dx = B.getX() - ax, dy = B.getY() - ay;
                    double segLen2 = dx * dx + dy * dy;
                    double t;
                    if (segLen2 < 1e-9) {
                        t = 0;
                    } else {
                        t = ((rx - ax) * dx + (ry - ay) * dy) / segLen2;
                        if (t < 0) t = 0;
                        else if (t > 1) t = 1;
                    }
                    double px = ax + t * dx;
                    double py = ay + t * dy;
                    double d = Math.hypot(rx - px, ry - py);
                    if (d < bestDist) {
                        bestDist = d;
                        bestEx = px;
                        bestEy = py;
                        bestEh = lerpAngle(A.getHeading(), B.getHeading(), t);
                        bestT = t;
                        bestA = i;
                        bestB = j;
                    }
                }
            }
            effX = bestEx;
            effY = bestEy;
            effHeading = bestEh;
            effDistance = bestDist;
            segA = bestA;
            segB = bestB;
            segT = bestT;
            nearestEndpoint = (bestT < 0.5) ? bestA : bestB;
        }

        double headingError = normalizeAngle(currentPose.getHeading() - effHeading);

        lastClosest = new Pose(effX, effY, effHeading);
        lastClosestIndex = nearestEndpoint;
        lastDistance = effDistance;
        lastHeadingErrorDeg = Math.toDegrees(headingError);
        lastPositionOk = effDistance < POSITION_TOLERANCE;
        lastHeadingOk = Math.abs(headingError) < HEADING_TOLERANCE;
        lastSegmentA = segA;
        lastSegmentB = segB;
        lastSegmentT = segT;

        if (effDistance > PROXIMITY_THRESHOLD) {
            deactivate();
            return false;
        }

        isActive = true;

        // t = 0 at the edge of range (red), 1 at the setpoint (green).
        double t = 1.0 - (effDistance / PROXIMITY_THRESHOLD);
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        currentR = (int) Math.round(255 * (1.0 - t));
        currentG = (int) Math.round(255 * t);
        currentB = 0;

        // Pulse only when BOTH position and heading are inside tolerance.
        boolean pulsing = lastPositionOk && lastHeadingOk;

        Color heat = new Color(currentR, currentG, currentB);
        boolean animationChanged = !wasActive || pulsing != wasPulsing;

        if (pulsing) {
            pulse.setPrimaryColor(heat);
            pulse.setSecondaryColor(new Color(0, 0, 0));
            if (animationChanged) {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, pulse);
            } else {
                prism.updateAnimationFromIndex(LayerHeight.LAYER_0);
            }
        } else {
            solid.setPrimaryColor(heat);
            if (animationChanged) {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solid);
            } else {
                prism.updateAnimationFromIndex(LayerHeight.LAYER_0);
            }
        }

        wasActive = true;
        wasPulsing = pulsing;
        return true;
    }

    // ==================== GETTERS ====================

    public boolean isActive() { return isActive; }
    public int getCurrentR() { return currentR; }
    public int getCurrentG() { return currentG; }
    public int getCurrentB() { return currentB; }

    public Pose getLastPose() { return lastPose; }
    public Pose getClosestShotPose() { return lastClosest; }
    public int getClosestShotIndex() { return lastClosestIndex; }
    public double getClosestDistance() { return lastDistance; }
    public double getHeadingErrorDeg() { return lastHeadingErrorDeg; }
    public boolean isPositionOk() { return lastPositionOk; }
    public boolean isHeadingOk() { return lastHeadingOk; }
    public boolean isShotReady() { return lastPositionOk && lastHeadingOk; }
    public int getSegmentAIndex() { return lastSegmentA; }
    public int getSegmentBIndex() { return lastSegmentB; }
    public double getSegmentT() { return lastSegmentT; }

    public void reset() {
        deactivate();
        currentR = 255;
        currentG = 0;
        currentB = 0;
    }

    // ==================== INTERNALS ====================

    private void deactivate() {
        isActive = false;
        wasActive = false;
        wasPulsing = false;
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private static double lerpAngle(double a, double b, double t) {
        // Lerp the short-way delta so wrapping 359°→1° interpolates correctly.
        return a + t * normalizeAngle(b - a);
    }
}
