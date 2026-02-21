package org.firstinspires.ftc.teamcode.SecretShenanigans;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * JController - Complete robot control abstraction
 * No need to touch Pedro Pathing directly - everything you need is here
 * Made by JADEN NOT ARI!!!
 */
public class JController {

    // ==================== HARDWARE REFERENCES ====================
    private Follower Follower;
    private DcMotor Intake;
    private DcMotor Catapult1;
    private DcMotor Catapult2;
    private Servo Foot1;
    private Servo Foot2;

    // ==================== TIMERS ====================
    private ElapsedTime ActionTimer = new ElapsedTime();

    // ==================== CONSTANTS ====================
    private static final double DEFAULT_TOLERANCE = 2.0;
    private static final double CATAPULT_UP_POWER = -1.0;
    private static final double CATAPULT_DOWN_POWER = 1.0;
    private static final double CATAPULT_HOLD_POWER = 0.0;
    private static final double CATAPULT_UP_DURATION = 0.35;
    private static final double CATAPULT_DOWN_DURATION = 0.4;
    private static final double INTAKE_IN_POWER = -1.0;
    private static final double INTAKE_OUT_POWER = 1.0;
    private static final double INTAKE_OFF_POWER = 0.0;
    private static final double FOOT_UP_POSITION = 0.5;
    private static final double FOOT_DOWN_POSITION = 0.175;

    // ==================== STATE TRACKING ====================
    private boolean IsNavigating = false;
    private boolean IsCatapultFiring = false;
    private Pose CurrentTarget = null;
    private double PositionTolerance = DEFAULT_TOLERANCE;
    private PathChainBuilder CurrentPathBuilder = null;

    // ==================== CONSTRUCTOR ====================
    public JController(Follower follower, DcMotor intake,
                       DcMotor catapult1, DcMotor catapult2,
                       Servo foot1, Servo foot2) {
        this.Follower = follower;
        this.Intake = intake;
        this.Catapult1 = catapult1;
        this.Catapult2 = catapult2;
        this.Foot1 = foot1;
        this.Foot2 = foot2;
    }

    // ==================== MAIN UPDATE METHOD ====================

    /**
     * Update all robot systems - CALL THIS EVERY LOOP!
     * Handles: follower updates, catapult sequences, navigation status
     */
    public void Update() {
        // Update follower
        Follower.update();

        // Update catapult firing sequence
        if (IsCatapultFiring) {
            double elapsed = ActionTimer.seconds();

            if (elapsed < CATAPULT_UP_DURATION) {
                // Up phase
                Catapult1.setPower(CATAPULT_UP_POWER);
                Catapult2.setPower(CATAPULT_UP_POWER);
            } else if (elapsed < CATAPULT_UP_DURATION + CATAPULT_DOWN_DURATION) {
                // Down phase
                Catapult1.setPower(CATAPULT_DOWN_POWER);
                Catapult2.setPower(CATAPULT_DOWN_POWER);
            } else {
                // Hold phase - finished
                Catapult1.setPower(CATAPULT_HOLD_POWER);
                Catapult2.setPower(CATAPULT_HOLD_POWER);
                IsCatapultFiring = false;
            }
        }

        // Auto-reset navigation flag when arrived
        if (IsNavigating && HasArrived()) {
            IsNavigating = false;
        }
    }

    // ==================== MOVEMENT METHODS ====================

    /**
     * Move to a specific position with heading
     * @param x X coordinate
     * @param y Y coordinate
     * @param heading Heading in radians
     */
    public void MoveToPosition(double x, double y, double heading) {
        MoveToPosition(new Pose(x, y, heading));
    }

    /**
     * Move to a specific pose
     * @param targetPose Target pose to navigate to
     */
    public void MoveToPosition(Pose targetPose) {
        CurrentTarget = targetPose;

        PathChain path = Follower.pathBuilder()
                .addPath(new Path(new BezierLine(Follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        Follower::getHeading, targetPose.getHeading(), 0.1f))
                .build();

        Follower.followPath(path);
        IsNavigating = true;
    }

    /**
     * Move to position with custom tolerance
     * @param x X coordinate
     * @param y Y coordinate
     * @param heading Heading in radians
     * @param tolerance Distance tolerance for arrival
     */
    public void MoveToPosition(double x, double y, double heading, double tolerance) {
        this.PositionTolerance = tolerance;
        MoveToPosition(x, y, heading);
    }

    /**
     * Override current path immediately and move to new position
     * @param x X coordinate
     * @param y Y coordinate
     * @param heading Heading in radians
     */
    public void OverrideAndMoveTo(double x, double y, double heading) {
        CancelNavigation();
        MoveToPosition(x, y, heading);
    }

    /**
     * Override current path immediately and move to new pose
     * @param targetPose Target pose
     */
    public void OverrideAndMoveTo(Pose targetPose) {
        CancelNavigation();
        MoveToPosition(targetPose);
    }

    /**
     * Move forward a specific distance
     * @param distance Distance in inches
     */
    public void MoveForward(double distance) {
        Pose currentPose = Follower.getPose();
        double heading = currentPose.getHeading();

        double newX = currentPose.getX() + distance * Math.cos(heading);
        double newY = currentPose.getY() + distance * Math.sin(heading);

        MoveToPosition(newX, newY, heading);
    }

    /**
     * Move backward a specific distance
     * @param distance Distance in inches
     */
    public void MoveBackward(double distance) {
        MoveForward(-distance);
    }

    /**
     * Strafe left a specific distance
     * @param distance Distance in inches
     */
    public void StrafeLeft(double distance) {
        Pose currentPose = Follower.getPose();
        double heading = currentPose.getHeading();

        double newX = currentPose.getX() - distance * Math.sin(heading);
        double newY = currentPose.getY() + distance * Math.cos(heading);

        MoveToPosition(newX, newY, heading);
    }

    /**
     * Strafe right a specific distance
     * @param distance Distance in inches
     */
    public void StrafeRight(double distance) {
        StrafeLeft(-distance);
    }

    /**
     * Turn to a specific heading
     * @param heading Target heading in radians
     */
    public void TurnToHeading(double heading) {
        Pose currentPose = Follower.getPose();
        MoveToPosition(currentPose.getX(), currentPose.getY(), heading);
    }

    /**
     * Turn to a specific heading in degrees
     * @param degrees Target heading in degrees
     */
    public void TurnToHeadingDegrees(double degrees) {
        TurnToHeading(Math.toRadians(degrees));
    }

    /**
     * Rotate by a specific angle
     * @param angleRadians Angle to rotate in radians (positive = counterclockwise)
     */
    public void Rotate(double angleRadians) {
        Pose currentPose = Follower.getPose();
        double newHeading = currentPose.getHeading() + angleRadians;
        MoveToPosition(currentPose.getX(), currentPose.getY(), newHeading);
    }

    /**
     * Rotate by a specific angle in degrees
     * @param angleDegrees Angle to rotate in degrees (positive = counterclockwise)
     */
    public void RotateDegrees(double angleDegrees) {
        Rotate(Math.toRadians(angleDegrees));
    }

    // ==================== PATH CHAIN BUILDER ====================

    /**
     * Start building a continuous path chain
     * @return PathChainBuilder for method chaining
     */
    public PathChainBuilder StartPath() {
        CurrentPathBuilder = new PathChainBuilder(this);
        return CurrentPathBuilder;
    }

    /**
     * Execute a pre-built PathChain
     * @param chain The PathChain to follow
     */
    public void FollowPathChain(PathChain chain) {
        Follower.followPath(chain);
        IsNavigating = true;
    }

    // ==================== PATH CHAIN BUILDER CLASS ====================

    public class PathChainBuilder {
        private JController controller;
        private List<Pose> waypoints;
        private double constantHeading;
        private boolean useConstantHeading;
        private boolean useTangentHeading;
        private double headingInterpolationStart;

        public PathChainBuilder(JController controller) {
            this.controller = controller;
            this.waypoints = new ArrayList<>();
            this.useConstantHeading = false;
            this.useTangentHeading = false;
            this.headingInterpolationStart = 0.1;

            // Add current position as first point
            waypoints.add(Follower.getPose());
        }

        /**
         * Add a waypoint to the path
         * @param x X coordinate
         * @param y Y coordinate
         * @param heading Heading in radians
         * @return this builder for chaining
         */
        public PathChainBuilder AddPoint(double x, double y, double heading) {
            waypoints.add(new Pose(x, y, heading));
            return this;
        }

        /**
         * Add a waypoint using a Pose
         * @param pose The pose to add
         * @return this builder for chaining
         */
        public PathChainBuilder AddPoint(Pose pose) {
            waypoints.add(pose);
            return this;
        }

        /**
         * Add multiple waypoints at once
         * @param poses Array of poses
         * @return this builder for chaining
         */
        public PathChainBuilder AddPoints(Pose... poses) {
            for (Pose pose : poses) {
                waypoints.add(pose);
            }
            return this;
        }

        /**
         * Move forward from current endpoint
         * @param distance Distance in inches
         * @return this builder for chaining
         */
        public PathChainBuilder Forward(double distance) {
            Pose last = waypoints.get(waypoints.size() - 1);
            double heading = last.getHeading();
            double newX = last.getX() + distance * Math.cos(heading);
            double newY = last.getY() + distance * Math.sin(heading);
            waypoints.add(new Pose(newX, newY, heading));
            return this;
        }

        /**
         * Move backward from current endpoint
         * @param distance Distance in inches
         * @return this builder for chaining
         */
        public PathChainBuilder Backward(double distance) {
            return Forward(-distance);
        }

        /**
         * Strafe left from current endpoint
         * @param distance Distance in inches
         * @return this builder for chaining
         */
        public PathChainBuilder StrafeLeft(double distance) {
            Pose last = waypoints.get(waypoints.size() - 1);
            double heading = last.getHeading();
            double newX = last.getX() - distance * Math.sin(heading);
            double newY = last.getY() + distance * Math.cos(heading);
            waypoints.add(new Pose(newX, newY, heading));
            return this;
        }

        /**
         * Strafe right from current endpoint
         * @param distance Distance in inches
         * @return this builder for chaining
         */
        public PathChainBuilder StrafeRight(double distance) {
            return StrafeLeft(-distance);
        }

        /**
         * Turn to heading at current position
         * @param heading Heading in radians
         * @return this builder for chaining
         */
        public PathChainBuilder TurnTo(double heading) {
            Pose last = waypoints.get(waypoints.size() - 1);
            waypoints.add(new Pose(last.getX(), last.getY(), heading));
            return this;
        }

        /**
         * Turn to heading in degrees at current position
         * @param degrees Heading in degrees
         * @return this builder for chaining
         */
        public PathChainBuilder TurnToDegrees(double degrees) {
            return TurnTo(Math.toRadians(degrees));
        }

        /**
         * Rotate by angle from current heading
         * @param angleRadians Angle in radians
         * @return this builder for chaining
         */
        public PathChainBuilder Rotate(double angleRadians) {
            Pose last = waypoints.get(waypoints.size() - 1);
            double newHeading = last.getHeading() + angleRadians;
            waypoints.add(new Pose(last.getX(), last.getY(), newHeading));
            return this;
        }

        /**
         * Rotate by angle in degrees from current heading
         * @param angleDegrees Angle in degrees
         * @return this builder for chaining
         */
        public PathChainBuilder RotateDegrees(double angleDegrees) {
            return Rotate(Math.toRadians(angleDegrees));
        }

        /**
         * Use constant heading throughout the path
         * @param heading Heading in radians
         * @return this builder for chaining
         */
        public PathChainBuilder WithConstantHeading(double heading) {
            this.constantHeading = heading;
            this.useConstantHeading = true;
            this.useTangentHeading = false;
            return this;
        }

        /**
         * Use constant heading in degrees throughout the path
         * @param degrees Heading in degrees
         * @return this builder for chaining
         */
        public PathChainBuilder WithConstantHeadingDegrees(double degrees) {
            return WithConstantHeading(Math.toRadians(degrees));
        }

        /**
         * Use tangent heading (face direction of travel)
         * @return this builder for chaining
         */
        public PathChainBuilder WithTangentHeading() {
            this.useTangentHeading = true;
            this.useConstantHeading = false;
            return this;
        }

        /**
         * Set heading interpolation start point
         * @param start Start point (0.0 to 1.0)
         * @return this builder for chaining
         */
        public PathChainBuilder SetHeadingInterpolationStart(double start) {
            this.headingInterpolationStart = start;
            return this;
        }

        /**
         * Create a circular path
         * @param centerX Center X coordinate
         * @param centerY Center Y coordinate
         * @param radius Radius in inches
         * @param numPoints Number of points around circle
         * @param facingInward True to face center, false to face tangent
         * @return this builder for chaining
         */
        public PathChainBuilder Circle(double centerX, double centerY, double radius,
                                       int numPoints, boolean facingInward) {
            Pose start = waypoints.get(waypoints.size() - 1);

            for (int i = 0; i < numPoints; i++) {
                double angle = (2 * Math.PI * i) / numPoints;
                double x = centerX + radius * Math.cos(angle);
                double y = centerY + radius * Math.sin(angle);

                double heading;
                if (facingInward) {
                    // Point toward center
                    double dx = centerX - x;
                    double dy = centerY - y;
                    heading = Math.atan2(dy, dx);
                } else {
                    // Point tangent to circle (direction of travel)
                    heading = angle + Math.PI / 2;
                }

                waypoints.add(new Pose(x, y, heading));
            }

            return this;
        }

        /**
         * Create an arc path
         * @param centerX Center X coordinate
         * @param centerY Center Y coordinate
         * @param radius Radius in inches
         * @param startAngleDegrees Starting angle in degrees
         * @param endAngleDegrees Ending angle in degrees
         * @param numPoints Number of points in arc
         * @return this builder for chaining
         */
        public PathChainBuilder Arc(double centerX, double centerY, double radius,
                                    double startAngleDegrees, double endAngleDegrees,
                                    int numPoints) {
            double startAngle = Math.toRadians(startAngleDegrees);
            double endAngle = Math.toRadians(endAngleDegrees);
            double angleStep = (endAngle - startAngle) / (numPoints - 1);

            for (int i = 0; i < numPoints; i++) {
                double angle = startAngle + angleStep * i;
                double x = centerX + radius * Math.cos(angle);
                double y = centerY + radius * Math.sin(angle);
                double heading = angle + Math.PI / 2; // Tangent to arc

                waypoints.add(new Pose(x, y, heading));
            }

            return this;
        }

        /**
         * Build and execute the path
         */
        public void Go() {
            if (waypoints.size() < 2) {
                return; // Need at least 2 points
            }

            com.pedropathing.paths.PathBuilder pathBuilder = Follower.pathBuilder();

            // Add paths between consecutive waypoints
            for (int i = 0; i < waypoints.size() - 1; i++) {
                Pose start = waypoints.get(i);
                Pose end = waypoints.get(i + 1);
                pathBuilder.addPath(new Path(new BezierLine(start, end)));
            }

            // Set heading interpolation
            if (useConstantHeading) {
                pathBuilder.setConstantHeadingInterpolation(constantHeading);
            } else if (useTangentHeading) {
                pathBuilder.setTangentHeadingInterpolation();
            } else {
                Pose lastPoint = waypoints.get(waypoints.size() - 1);
                pathBuilder.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        Follower::getHeading, lastPoint.getHeading(), headingInterpolationStart));
            }

            PathChain chain = pathBuilder.build();
            controller.FollowPathChain(chain);
            controller.CurrentTarget = waypoints.get(waypoints.size() - 1);
        }

        /**
         * Build the PathChain without executing it
         * @return The built PathChain
         */
        public PathChain Build() {
            if (waypoints.size() < 2) {
                return null;
            }

            com.pedropathing.paths.PathBuilder pathBuilder = Follower.pathBuilder();

            for (int i = 0; i < waypoints.size() - 1; i++) {
                Pose start = waypoints.get(i);
                Pose end = waypoints.get(i + 1);
                pathBuilder.addPath(new Path(new BezierLine(start, end)));
            }

            if (useConstantHeading) {
                pathBuilder.setConstantHeadingInterpolation(constantHeading);
            } else if (useTangentHeading) {
                pathBuilder.setTangentHeadingInterpolation();
            } else {
                Pose lastPoint = waypoints.get(waypoints.size() - 1);
                pathBuilder.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        Follower::getHeading, lastPoint.getHeading(), headingInterpolationStart));
            }

            controller.CurrentTarget = waypoints.get(waypoints.size() - 1);
            return pathBuilder.build();
        }
    }

    // ==================== CATAPULT METHODS ====================

    /**
     * Fire the catapult with default timing
     * Non-blocking - Update() will handle the sequence automatically
     */
    public void FireCatapult() {
        Catapult1.setPower(CATAPULT_UP_POWER);
        Catapult2.setPower(CATAPULT_UP_POWER);
        IsCatapultFiring = true;
        ActionTimer.reset();
    }

    /**
     * Manually control catapult
     * @param power Power level (-1 to 1)
     */
    public void SetCatapultPower(double power) {
        Catapult1.setPower(power);
        Catapult2.setPower(power);
    }

    /**
     * Hold catapult in current position
     */
    public void HoldCatapult() {
        Catapult1.setPower(CATAPULT_HOLD_POWER);
        Catapult2.setPower(CATAPULT_HOLD_POWER);
        IsCatapultFiring = false;
    }

    // ==================== INTAKE METHODS ====================

    /**
     * Run intake to collect game elements
     */
    public void IntakeIn() {
        Intake.setPower(INTAKE_IN_POWER);
    }

    /**
     * Reverse intake to eject game elements
     */
    public void IntakeOut() {
        Intake.setPower(INTAKE_OUT_POWER);
    }

    /**
     * Stop intake
     */
    public void IntakeStop() {
        Intake.setPower(INTAKE_OFF_POWER);
    }

    /**
     * Set intake power manually
     * @param power Power level (-1 to 1)
     */
    public void SetIntakePower(double power) {
        Intake.setPower(power);
    }

    // ==================== FOOT/SERVO METHODS ====================

    /**
     * Raise the feet
     */
    public void FeetUp() {
        Foot1.setPosition(FOOT_UP_POSITION);
        Foot2.setPosition(FOOT_UP_POSITION);
    }

    /**
     * Lower the feet
     */
    public void FeetDown() {
        Foot1.setPosition(FOOT_DOWN_POSITION);
        Foot2.setPosition(FOOT_DOWN_POSITION);
    }

    /**
     * Set foot positions manually
     * @param position Position (0.0 to 1.0)
     */
    public void SetFeetPosition(double position) {
        Foot1.setPosition(position);
        Foot2.setPosition(position);
    }

    // ==================== COMPOUND ACTIONS ====================

    /**
     * Move to position and fire catapult when arrived
     * @param x X coordinate
     * @param y Y coordinate
     * @param heading Heading in radians
     */
    public void MoveAndFire(double x, double y, double heading) {
        MoveToPosition(x, y, heading);
    }

    /**
     * Move forward while intaking
     * @param distance Distance to move
     */
    public void MoveForwardAndIntake(double distance) {
        IntakeIn();
        MoveForward(distance);
    }

    /**
     * Park sequence - move to position and lower feet
     * @param x X coordinate
     * @param y Y coordinate
     * @param heading Heading in radians
     */
    public void Park(double x, double y, double heading) {
        MoveToPosition(x, y, heading);
    }

    // ==================== MATH UTILITIES ====================

    /**
     * Calculate distance between two points
     * @param x1 First point X
     * @param y1 First point Y
     * @param x2 Second point X
     * @param y2 Second point Y
     * @return Distance in inches
     */
    public static double Distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    /**
     * Calculate angle from point 1 to point 2
     * @param x1 First point X
     * @param y1 First point Y
     * @param x2 Second point X
     * @param y2 Second point Y
     * @return Angle in radians
     */
    public static double AngleTo(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1, x2 - x1);
    }

    /**
     * Convert degrees to radians
     * @param degrees Angle in degrees
     * @return Angle in radians
     */
    public static double ToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * Convert radians to degrees
     * @param radians Angle in radians
     * @return Angle in degrees
     */
    public static double ToDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Normalize angle to -PI to PI range
     * @param angle Angle in radians
     * @return Normalized angle
     */
    public static double NormalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Calculate shortest angular difference
     * @param from Starting angle in radians
     * @param to Ending angle in radians
     * @return Shortest difference in radians
     */
    public static double AngleDifference(double from, double to) {
        double diff = to - from;
        return NormalizeAngle(diff);
    }

    /**
     * Clamp a value between min and max
     * @param value Value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped value
     */
    public static double Clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ==================== STATUS METHODS ====================

    /**
     * Check if robot has arrived at target position
     * @return true if at target position
     */
    public boolean HasArrived() {
        if (!IsNavigating || CurrentTarget == null) return true;

        Pose currentPose = Follower.getPose();
        double distance = Math.hypot(
                currentPose.getX() - CurrentTarget.getX(),
                currentPose.getY() - CurrentTarget.getY()
        );

        return distance < PositionTolerance && !Follower.isBusy();
    }

    /**
     * Check if robot is currently navigating
     * @return true if navigating to a position
     */
    public boolean IsNavigating() {
        return IsNavigating;
    }

    /**
     * Check if catapult is currently firing
     * @return true if catapult sequence is running
     */
    public boolean IsCatapultBusy() {
        return IsCatapultFiring;
    }

    /**
     * Get distance to current target
     * @return distance in inches, or -1 if no target
     */
    public double GetDistanceToTarget() {
        if (CurrentTarget == null) return -1;

        Pose currentPose = Follower.getPose();
        return Math.hypot(
                currentPose.getX() - CurrentTarget.getX(),
                currentPose.getY() - CurrentTarget.getY()
        );
    }

    /**
     * Get current robot pose
     * @return Current pose
     */
    public Pose GetCurrentPose() {
        return Follower.getPose();
    }

    /**
     * Get current X position
     * @return X coordinate in inches
     */
    public double GetX() {
        return Follower.getPose().getX();
    }

    /**
     * Get current Y position
     * @return Y coordinate in inches
     */
    public double GetY() {
        return Follower.getPose().getY();
    }

    /**
     * Get current heading
     * @return Heading in radians
     */
    public double GetHeading() {
        return Follower.getPose().getHeading();
    }

    /**
     * Get current heading in degrees
     * @return Heading in degrees
     */
    public double GetHeadingDegrees() {
        return Math.toDegrees(Follower.getPose().getHeading());
    }

    /**
     * Cancel current navigation
     */
    public void CancelNavigation() {
        Follower.startTeleopDrive();
        IsNavigating = false;
        CurrentTarget = null;
        PositionTolerance = DEFAULT_TOLERANCE;
    }

    /**
     * Stop all motors and servos
     */
    public void StopAll() {
        CancelNavigation();
        IntakeStop();
        HoldCatapult();
    }

    /**
     * Set default position tolerance
     * @param tolerance Tolerance in inches
     */
    public void SetDefaultTolerance(double tolerance) {
        this.PositionTolerance = tolerance;
    }

    /**
     * Emergency stop - immediately halt all movement
     */
    public void EmergencyStop() {
        Follower.breakFollowing();
        StopAll();
    }
}