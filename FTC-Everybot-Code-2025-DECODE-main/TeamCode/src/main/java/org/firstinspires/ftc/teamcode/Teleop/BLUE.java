package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Limelight.LimelightPoseUpdater;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Prism.HeatMapController;
import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Util.VelocityShotController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
@TeleOp(name = "BLUE TELEOP", group = "Teleop")
public class BLUE extends OpMode {

    // ==================== LED COLOR TRACKING ====================
    private boolean hasInitializedPoseFromLimelight = false;
    private boolean hasAcceptedFirstLimelightPose = false;
    private LimelightPoseUpdater limelightPoseUpdater = null;

    private ElapsedTime ledUpdateTimer = new ElapsedTime();
    private static final double LED_UPDATE_INTERVAL = 0.03;

    // ==================== STATE MACHINE ENUMS ====================
private enum RobotState {
        STARTUP,
        MANUAL_DRIVE,
        VELOCITY_SHOT_ACTIVE,
        NAVIGATING_TO_VELOCITY_SHOT,
        NAVIGATING_TO_VELOCITY_SCORE,
        NAVIGATING_TO_SETPOINT,
        NAVIGATING_TO_TARGET_ONE,
        NAVIGATING_TO_TARGET_TWO,
        NAVIGATING_TO_GATE,
        NAVIGATING_TO_PARK,
        LAUNCHING_UP,
        LAUNCHING_DOWN,
        LAUNCHING_HOLD
    }

    private enum CatapultModes {UP, DOWN, HOLD}
    private enum FootMode {UP, DOWN}

    private enum LedMode {
        SWIRL,
        INTAKE_REVERSE,
        INTAKE,
        SOLID_BLUE,
        SOLID_WHITE,
        BLINK,
        BLUE_WAVE,
        HEATMAP
    }

    private LedMode currentLedMode = null;

    // ==================== STATE VARIABLES ====================
    private RobotState currentState = RobotState.STARTUP;
    private RobotState previousState = RobotState.STARTUP;

    // ==================== TIMERS ====================
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime startupCatapultTimer = new ElapsedTime();

    // ==================== HARDWARE ====================
    private Follower follower;
    private TelemetryManager telemetryM;
    private DcMotor intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private Servo foot1 = null;
    private Servo foot2 = null;

    private Limelight3A camera;
    private LimelightPoseUpdater limelightUpdater = null;

    // ==================== VELOCITY SHOT CONTROLLER ====================
    private VelocityShotController velocityShotController;
    private double velocityShotTolerance = 50;

    // ==================== HEATMAP LED CONTROLLER ====================
    private HeatMapController heatMapController;

    // ==================== NAVIGATION SETPOINTS ====================
    public static Pose startingPose;
    private Pose VelocityShotSetpoint = new Pose(150, 148, Math.toRadians(37)).mirror();
    private Pose targetSetpoint = new Pose(27.021671826625383, 125.25077399380805, Math.toRadians(142));
    private Pose targetOneSetpoint = new Pose(102.34525660964229, 110.63141524105754, Math.toRadians(37)).mirror();
    private Pose targetTwoSetpoint = new Pose(112.64696734059099, 119.58942457231727, Math.toRadians(37)).mirror();
    private Pose gateSetpoint = new Pose(128.657, 72, Math.toRadians(90)).mirror();
    private Pose gateWaypoint = new Pose(120.73170731707316, 72.5, Math.toRadians(90)).mirror();
    private Pose parkPose = new Pose(43.474387184684886, 39.59050310801249, Math.toRadians(230)).mirror();

    // ==================== CONSTANTS ====================
    private static final double SETPOINT_TOLERANCE = 2;
    private static final double LAUNCH_HEADING_TOLERANCE = Math.toRadians(5);
    private static final double LAUNCH_MAX_LINEAR_SPEED = 4.0;   // in/s
    private static final double LAUNCH_MAX_ANGULAR_SPEED = 0.7;  // rad/s
    private static final double LAUNCH_UP_DURATION = 0.2;
    private static final double LAUNCH_DOWN_DURATION = 0.4;
    private static final double STARTUP_DOWN_DURATION = 0.25;

    private double INTAKE_IN_POWER = -0.8;
    private double INTAKE_OUT_POWER = 1.0;
    private double INTAKE_OFF_POWER = 0.0;
    private double FOOT_UP_POSITION = 0.5;
    private double FOOT_DOWN_POSITION = 0.1750;
    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
    private double CATAPULT_HOLD_POWER = 0.0;

    // ==================== CONTROL VARIABLES ====================
    private boolean wasFootUpPressed = false;
    private boolean wasFootDownPressed = false;
    private CatapultModes pivotMode;
    private FootMode footmode;
    private boolean isHoldingPosition = false;
    private Pose lastHoldPose = null;
    private boolean isHeadingLockActive = false;
    private boolean wasHeadingLockButtonPressed = false;
    private static final double DRIVER_DEADZONE = 0.1;
    private static final double HOLD_CAPTURE_SPEED = 1.5;        // in/s
    private static final double HOLD_CAPTURE_ANGULAR_SPEED = 0.5; // rad/s
    private static final double HOLD_DISTURBANCE_DISTANCE = 7.0;          // in
    private static final double HOLD_DISTURBANCE_HEADING = Math.toRadians(30);
    private static final double HEADING_LOCK_ANGLE = Math.toRadians(90);  // angle to snap to while B held
    // Heading-lock controller — uses the same PIDF coefficients tuned in
    // pedroPathing/Constants.java for the follower's heading loop.
    private final PIDFController headingLockController =
            new PIDFController(new PIDFCoefficients(0.6, 0, 0, 0));

    // ==================== LED VARIABLES ====================
    private GoBildaPrismDriver prism = null;
    private PrismAnimations.Solid solidRed = new PrismAnimations.Solid(Color.RED);
    private PrismAnimations.Solid solidBlue = new PrismAnimations.Solid(Color.BLUE);
    private PrismAnimations.Solid solidWhite = new PrismAnimations.Solid(Color.WHITE);
    private PrismAnimations.Solid solidCyan = new PrismAnimations.Solid(Color.CYAN);
    private PrismAnimations.Blink blinkBlueWhite = new PrismAnimations.Blink(Color.BLUE, Color.WHITE);
    private PrismAnimations.Snakes swirlAnimation = new PrismAnimations.Snakes(Color.BLUE, Color.CYAN, Color.WHITE);
    private PrismAnimations.SineWave blueWaveAnimation = new PrismAnimations.SineWave(Color.BLUE, Color.TRANSPARENT);

    private boolean isIntakeRunning = false;
    private boolean isIntakeReversing = false;
    private boolean wasIntakeInPressed = false;
    private boolean wasIntakeOutPressed = false;
    private boolean feetButton = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");
        foot1 = hardwareMap.get(Servo.class, "foot1");
        foot2 = hardwareMap.get(Servo.class, "foot2");
        camera = hardwareMap.get(Limelight3A.class, "limelight");

        intake.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        foot1.setDirection(Servo.Direction.FORWARD);
        foot2.setDirection(Servo.Direction.REVERSE);

        foot1.setPosition(FOOT_UP_POSITION);
        foot2.setPosition(FOOT_UP_POSITION);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        // Initialize velocity shot controller
        velocityShotController = new VelocityShotController(
                VelocityShotSetpoint.getX(),
                VelocityShotSetpoint.getY(),
                VelocityShotSetpoint.getHeading()
        );

        // Initialize HeatMap controller and register known good shooting positions.
        // Drive the robot to a spot that makes shots, read the "Pose" telemetry,
        // then copy those numbers in here as a new addShotPose(...) call.
        heatMapController = new HeatMapController(prism);
        // Shot poses are given in RED-frame coordinates and .mirror()ed to the blue side,
        // same convention as the navigation setpoints above — that way a pose recorded on
        // the red robot drops in here verbatim.
        heatMapController.addShotPose(new Pose(114.4184, 132.0035, Math.toRadians(12.82)).mirror());
        heatMapController.addShotPose(new Pose(116.4101, 122.9897, Math.toRadians(25.43)).mirror());
        heatMapController.addShotPose(new Pose(119.3829, 125.1452, Math.toRadians(16.94)).mirror());
        heatMapController.addShotPose(new Pose(118.9329, 127.6452, Math.toRadians(17.06)).mirror());
        heatMapController.addShotPose(new Pose(118.7867, 130.0832, Math.toRadians(23.46)).mirror());
        heatMapController.addShotPose(new Pose(118.5060, 130.0316, Math.toRadians(28.96)).mirror());
        heatMapController.addShotPose(new Pose(122.3506, 128.2685, Math.toRadians(30.23)).mirror());
        heatMapController.addShotPose(new Pose(124.2448, 125.5624, Math.toRadians(29.34)).mirror());
        heatMapController.addShotPose(new Pose(123.6228, 118.8057, Math.toRadians(46.40)).mirror());
        heatMapController.addShotPose(new Pose(123.8874, 119.6107, Math.toRadians(46.08)).mirror());
        heatMapController.addShotPose(new Pose(123.5328, 122.5747, Math.toRadians(42.67)).mirror());
        heatMapController.addShotPose(new Pose(114.6971, 129.0082, Math.toRadians(20.53)).mirror());
        heatMapController.addShotPose(new Pose(118.6083, 118.8968, Math.toRadians(32.71)).mirror());
        heatMapController.addShotPose(new Pose(118.5951, 120.0957, Math.toRadians(30.18)).mirror());
        heatMapController.addShotPose(new Pose(112.1078, 125.5669, Math.toRadians(25.59)).mirror());
        heatMapController.addShotPose(new Pose(120.6168, 123.0275, Math.toRadians(43.28)).mirror());
        heatMapController.addShotPose(new Pose(119.5159, 127.1542, Math.toRadians(39.01)).mirror());
        heatMapController.addShotPose(new Pose(120.1838, 117.4008, Math.toRadians(51.04)).mirror());
        heatMapController.addShotPose(new Pose(119.4322, 122.8069, Math.toRadians(35.91)).mirror());
        heatMapController.addShotPose(new Pose(120.6674, 122.1943, Math.toRadians(30.05)).mirror());
        heatMapController.addShotPose(new Pose(119.9719, 127.8185, Math.toRadians(24.27)).mirror());
        heatMapController.addShotPose(new Pose(117.5773, 129.8076, Math.toRadians(16.27)).mirror());
        heatMapController.addShotPose(new Pose(117.6494, 127.7719, Math.toRadians(8.41)).mirror());
        heatMapController.addShotPose(new Pose(115.0767, 133.7463, Math.toRadians(0.43)).mirror());
        heatMapController.addShotPose(new Pose(123.7709, 124.9373, Math.toRadians(42.67)).mirror());
        heatMapController.addShotPose(new Pose(120.3936, 125.1025, Math.toRadians(33.92)).mirror());
        heatMapController.addShotPose(new Pose(120.3229, 120.0176, Math.toRadians(43.23)).mirror());
        heatMapController.addShotPose(new Pose(126.0496, 114.5735, Math.toRadians(56.26)).mirror());
        heatMapController.addShotPose(new Pose(125.2238, 118.1025, Math.toRadians(61.06)).mirror());
        heatMapController.addShotPose(new Pose(119.8480, 124.7540, Math.toRadians(43.36)).mirror());
        heatMapController.addShotPose(new Pose(119.59, 126.45, Math.toRadians(7.72)).mirror());

        // heatMapController.addShotPose(new Pose(X, Y, Math.toRadians(HEADING_DEG)).mirror());

        // Configure LED animations
        solidRed.setBrightness(100);
        solidBlue.setBrightness(100);
        solidWhite.setBrightness(100);
        solidCyan.setBrightness(100);

        blinkBlueWhite.setBrightness(100);
        blinkBlueWhite.setPeriod(300);
        blinkBlueWhite.setPrimaryColorPeriod(150);

        swirlAnimation.setSpeed(0.6f);
        swirlAnimation.setSnakeLength(8);
        swirlAnimation.setSpacingBetween(3);
        swirlAnimation.setRepeatAfter(20);
        swirlAnimation.setBackgroundColor(Color.TRANSPARENT);
        swirlAnimation.setBrightness(100);

        blueWaveAnimation.setPeriod(1200);
        blueWaveAnimation.setSpeed(0.4f);
        blueWaveAnimation.setOffset(0.5f);
        blueWaveAnimation.setBrightness(100);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        runtime.reset();
        camera.start();
        stateTimer.reset();
        ledUpdateTimer.reset();
        changeState(RobotState.STARTUP);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        LLResult result = camera.getLatestResult();
        Pose currentPose = follower.getPose();

        if (result != null && result.isValid() && result.getBotposeTagCount() > 0) {
            Pose3D botpose3D = result.getBotpose();
            if (botpose3D != null) {
                Vector velocity = follower.getVelocity();
                double angularVelocity = follower.getAngularVelocity();
                double linearVelocity = Math.hypot(
                        velocity.getXComponent(),
                        velocity.getYComponent()
                );

                // Overriding pose while a path is being followed makes the controller
                // see a teleport and jerk. Only correct when the driver is in control.
                boolean stationary = linearVelocity < 1.8 && Math.abs(angularVelocity) < 1.8;
                if (stationary && !isNavigatingState()) {
                    Pose limelightPose = LimelightPoseUpdater.convertLimelightToPedro(
                            botpose3D.getPosition().x,
                            botpose3D.getPosition().y,
                            currentPose.getHeading()
                    );

                    follower.setPose(limelightPose);
                }
            }
        }

        heatMapController.update(follower);

        // Update LEDs
        updateLEDs();

        handleGlobalInput();

        switch (currentState) {
            case STARTUP:
                handleStartup();
                break;
            case MANUAL_DRIVE:
                handleManualDrive();
                break;
            case NAVIGATING_TO_PARK:
                handleNavigatingToPark();
                break;
            case NAVIGATING_TO_VELOCITY_SHOT:
                handleNavigatingToVelocityShot();
                break;
            case NAVIGATING_TO_SETPOINT:
                handleNavigatingToSetpoint();
                break;
            case NAVIGATING_TO_TARGET_ONE:
                handleNavigatingToTargetOne();
                break;
            case NAVIGATING_TO_TARGET_TWO:
                handleNavigatingToTargetTwo();
                break;
            // Disabled — B is now heading lock, no path triggers this state.
            // case NAVIGATING_TO_GATE:
            //     handleNavigatingToGate();
            //     break;
            case LAUNCHING_UP:
                handleLaunchingUp();
                break;
            case LAUNCHING_DOWN:
                handleLaunchingDown();
                break;
            case LAUNCHING_HOLD:
                handleLaunchingHold();
                break;
        }

        updateTelemetry();
    }

    // ==================== LED UPDATE SYSTEM ====================

    private boolean heatmapOwnedLayerLastFrame = false;

    private void updateLEDs() {
        // When the heatmap is driving the LEDs, step aside. When it hands control
        // back, invalidate currentLedMode so the normal logic re-asserts its choice.
        if (heatMapController.isActive()) {
            currentLedMode = LedMode.HEATMAP;
            heatmapOwnedLayerLastFrame = true;
            return;
        }

        if (heatmapOwnedLayerLastFrame) {
            currentLedMode = null;
            heatmapOwnedLayerLastFrame = false;
        }

        updateNormalLEDs();
    }

    private void updateNormalLEDs() {
        if (feetButton) {
            setLedMode(LedMode.SWIRL);
            return;
        }

        if (isIntakeReversing) {
            setLedMode(LedMode.INTAKE_REVERSE);
            return;
        }

        if (isIntakeRunning) {
            setLedMode(LedMode.INTAKE);
            return;
        }

        switch (currentState) {
            case STARTUP:
                setLedMode(LedMode.SOLID_WHITE);
                break;

            case MANUAL_DRIVE:
            case LAUNCHING_UP:
            case LAUNCHING_DOWN:
            case LAUNCHING_HOLD:
                setLedMode(LedMode.BLUE_WAVE);
                break;

            default:
                setLedMode(LedMode.BLUE_WAVE);
                break;
        }
    }

    private void setLedMode(LedMode newMode) {
        if (newMode == currentLedMode) return;

        currentLedMode = newMode;

        switch (newMode) {
            case SWIRL:
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, swirlAnimation);
                break;
            case INTAKE_REVERSE:
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidRed);
                break;
            case INTAKE:
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidCyan);
                break;
            case SOLID_BLUE:
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidBlue);
                break;
            case SOLID_WHITE:
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidWhite);
                break;
            case BLINK:
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, blinkBlueWhite);
                break;
            case BLUE_WAVE:
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, blueWaveAnimation);
                break;
            case HEATMAP:
                // Handled by HeatMapController
                break;
        }
    }



    // ==================== HELPER METHODS ====================

    private boolean isNavigatingState() {
        return currentState == RobotState.VELOCITY_SHOT_ACTIVE ||
                currentState == RobotState.NAVIGATING_TO_VELOCITY_SHOT ||
                currentState == RobotState.NAVIGATING_TO_VELOCITY_SCORE ||
                currentState == RobotState.NAVIGATING_TO_SETPOINT ||
                currentState == RobotState.NAVIGATING_TO_TARGET_ONE ||
                currentState == RobotState.NAVIGATING_TO_TARGET_TWO ||
                currentState == RobotState.NAVIGATING_TO_GATE;
    }

    private Pose getActiveGoalPose() {
        switch (currentState) {
            case VELOCITY_SHOT_ACTIVE:
            case NAVIGATING_TO_VELOCITY_SHOT:
                return VelocityShotSetpoint;
            case NAVIGATING_TO_VELOCITY_SCORE:
            case NAVIGATING_TO_SETPOINT:
                return targetSetpoint;
            case NAVIGATING_TO_TARGET_ONE:
                return targetOneSetpoint;
            case NAVIGATING_TO_TARGET_TWO:
                return targetTwoSetpoint;
            case NAVIGATING_TO_GATE:
                return gateSetpoint;
            case NAVIGATING_TO_PARK:
                return parkPose;
            default:
                return targetSetpoint;
        }
    }

    private double calculateDistanceToGoal() {
        Pose goalPose = getActiveGoalPose();
        double dx = follower.getPose().getX() - goalPose.getX();
        double dy = follower.getPose().getY() - goalPose.getY();
        return Math.hypot(dx, dy);
    }

    // Fires a launch the instant distance, heading, and velocity are all in tolerance —
    // don't wait on follower.isBusy() to settle, but don't shoot while still moving.
    private boolean isReadyToLaunch(Pose target) {
        Pose pose = follower.getPose();
        double distance = Math.hypot(pose.getX() - target.getX(), pose.getY() - target.getY());
        double rawErr = pose.getHeading() - target.getHeading();
        double headingErr = Math.atan2(Math.sin(rawErr), Math.cos(rawErr));

        Vector v = follower.getVelocity();
        double speed = Math.hypot(v.getXComponent(), v.getYComponent());
        double angularSpeed = Math.abs(follower.getAngularVelocity());

        return distance < SETPOINT_TOLERANCE
                && Math.abs(headingErr) < LAUNCH_HEADING_TOLERANCE
                && speed < LAUNCH_MAX_LINEAR_SPEED
                && angularSpeed < LAUNCH_MAX_ANGULAR_SPEED;
    }

    // ==================== STATE HANDLERS ====================

    private void handleStartup() {
        if (stateTimer.seconds() < STARTUP_DOWN_DURATION) {
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
            pivotMode = CatapultModes.DOWN;
        } else {
            catapult1.setPower(CATAPULT_HOLD_POWER);
            catapult2.setPower(CATAPULT_HOLD_POWER);
            pivotMode = CatapultModes.HOLD;
            changeState(RobotState.MANUAL_DRIVE);
        }
    }

    private void handleManualDrive() {
        if (gamepad1.right_trigger > 0.2) {
            changeState(RobotState.LAUNCHING_UP);
            return;
        }

        // Right Bumper + Y = velocity shot with tolerance 50
        if (gamepad1.right_bumper && gamepad1.y) {
            velocityShotTolerance = 90;
            startVelocityShotController();
            return;
        }

        // Right Bumper + X = velocity shot with tolerance 80
        if (gamepad1.right_bumper && gamepad1.x) {
            velocityShotTolerance = 110;
            startVelocityShotController();
            return;
        }

        // Right Bumper alone does nothing (no action here)

        if (gamepad1.dpad_right) {
            startNavigationToPark();
            return;
        }
        if (gamepad1.y) {
            startNavigationToSetpoint();
            return;
        }
        if (gamepad1.a) {
            startNavigationToTargetOne();
            return;
        }
        if (gamepad1.x) {
            startNavigationToTargetTwo();
            return;
        }
        // B / Circle is now heading lock — handled below, not here.
        // if (gamepad1.b) {
        //     startNavigationToGate();
        //     return;
        // }

        boolean driverInput = Math.abs(gamepad1.left_stick_y) > DRIVER_DEADZONE ||
                Math.abs(gamepad1.left_stick_x) > DRIVER_DEADZONE ||
                Math.abs(gamepad1.right_stick_x) > DRIVER_DEADZONE;

        // Toggle heading lock on the rising edge of B.
        if (gamepad1.b && !wasHeadingLockButtonPressed) {
            isHeadingLockActive = !isHeadingLockActive;
        }
        wasHeadingLockButtonPressed = gamepad1.b;

        if (isHeadingLockActive) {
            // Heading lock: ignore right stick, let PedroPathing's PIDF controller
            // drive heading toward HEADING_LOCK_ANGLE.
            if (isHoldingPosition) {
                follower.startTeleopDrive();
                isHoldingPosition = false;
            }
            double err = HEADING_LOCK_ANGLE - follower.getPose().getHeading();
            err = Math.atan2(Math.sin(err), Math.cos(err));
            headingLockController.updateError(err);
            double rotation = Math.max(-1.0, Math.min(1.0, headingLockController.run()));
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    rotation,
                    false
            );
        } else if (driverInput) {
            if (isHoldingPosition) {
                follower.startTeleopDrive();
                isHoldingPosition = false;
            }
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        } else if (isHoldingPosition) {
            // If someone shoved the bot off the hold point, give up fighting and let
            // it settle — otherwise the PID keeps driving "on its own" to return.
            Pose p = follower.getPose();
            double displacement = Math.hypot(
                    p.getX() - lastHoldPose.getX(),
                    p.getY() - lastHoldPose.getY()
            );
            double rawHeadingErr = p.getHeading() - lastHoldPose.getHeading();
            double headingErr = Math.atan2(Math.sin(rawHeadingErr), Math.cos(rawHeadingErr));
            if (displacement > HOLD_DISTURBANCE_DISTANCE
                    || Math.abs(headingErr) > HOLD_DISTURBANCE_HEADING) {
                follower.startTeleopDrive();
                isHoldingPosition = false;
            }
        } else {
            // Coast to rest before snapping the hold pose — capturing while the bot
            // still has momentum makes the PID yank it backward.
            Vector v = follower.getVelocity();
            double speed = Math.hypot(v.getXComponent(), v.getYComponent());
            if (speed < HOLD_CAPTURE_SPEED && Math.abs(follower.getAngularVelocity()) < HOLD_CAPTURE_ANGULAR_SPEED) {
                startHoldingPosition();
            } else {
                follower.setTeleOpDrive(0, 0, 0, false);
            }
        }

        handleManualControls();
    }

    private void startHoldingPosition() {
        Pose holdPose = follower.getPose();
        follower.holdPoint(holdPose);
        lastHoldPose = holdPose;
        isHoldingPosition = true;
    }

    private void handleNavigatingToSetpoint() {
        if (checkForDriverInterruption()) return;

        if (isReadyToLaunch(targetSetpoint)) {
            changeState(RobotState.LAUNCHING_UP);
        }
    }

    private void handleNavigatingToPark() {
        if (checkForDriverInterruption()) return;

        double distanceToTarget = Math.hypot(
                follower.getPose().getX() - parkPose.getX(),
                follower.getPose().getY() - parkPose.getY()
        );

        if (distanceToTarget < 2 && !follower.isBusy()) {
            // Lower the feet
            foot1.setPosition(FOOT_DOWN_POSITION);
            foot2.setPosition(FOOT_DOWN_POSITION);
            changeState(RobotState.MANUAL_DRIVE);
        }
    }

    private void handleNavigatingToTargetOne() {
        if (checkForDriverInterruption()) return;

        if (isReadyToLaunch(targetOneSetpoint)) {
            changeState(RobotState.LAUNCHING_UP);
        }
    }

    private void handleNavigatingToTargetTwo() {
        if (checkForDriverInterruption()) return;

        if (isReadyToLaunch(targetTwoSetpoint)) {
            changeState(RobotState.LAUNCHING_UP);
        }
    }

    // Disabled — B is now heading lock.
    private void handleNavigatingToGate() {
        // if (checkForDriverInterruption()) return;
        //
        // double distanceToTarget = Math.hypot(
        //         follower.getPose().getX() - gateSetpoint.getX(),
        //         follower.getPose().getY() - gateSetpoint.getY()
        // );
        //
        // if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
        //     changeState(RobotState.MANUAL_DRIVE);
        // }
    }

    private void handleLaunchingUp() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);

        handleIntakeDuringLaunch();

        if (stateTimer.seconds() >= LAUNCH_UP_DURATION) {
            // Ball has left the catapult — hand the driver control for the down-stroke.
            follower.startTeleopDrive();
            changeState(RobotState.LAUNCHING_DOWN);
        }
    }

    private void handleLaunchingDown() {
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);

        handleIntakeDuringLaunch();

        // Drive normally while the catapult resets.
        follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        if (stateTimer.seconds() >= LAUNCH_DOWN_DURATION) {
            changeState(RobotState.LAUNCHING_HOLD);
        }
    }

    private void handleLaunchingHold() {
        catapult1.setPower(CATAPULT_HOLD_POWER);
        catapult2.setPower(CATAPULT_HOLD_POWER);
        changeState(RobotState.MANUAL_DRIVE);
    }

    // ==================== HELPER METHODS ====================

    private void changeState(RobotState newState) {
        previousState = currentState;
        currentState = newState;
        stateTimer.reset();
        if (newState != RobotState.MANUAL_DRIVE) {
            isHoldingPosition = false;
            isHeadingLockActive = false;
        }
    }

    private void handleGlobalInput() {

        if (gamepad1.back) {
            Pose currentPose = follower.getPose();
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(180)));
        }
    }

    private boolean checkForDriverInterruption() {
        boolean driverInput = Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1;

        if (driverInput) {
            follower.startTeleopDrive();
            changeState(RobotState.MANUAL_DRIVE);
            return true;
        }
        return false;
    }

    private void handleIntakeInJustPressed() {
        isIntakeRunning = true;
        isIntakeReversing = false;
    }

    private void handleIntakeOutJustPressed() {
        isIntakeRunning = false;
        isIntakeReversing = true;
    }

    private static final double FOOT_EPSILON = 0.02;

    private boolean isFootAllTheWayOut() {
        return Math.abs(foot1.getPosition() - FOOT_DOWN_POSITION) < FOOT_EPSILON
                && Math.abs(foot2.getPosition() - FOOT_DOWN_POSITION) < FOOT_EPSILON;
    }

    private void handleManualControls() {
        boolean intakeInButton = gamepad1.left_trigger > 0.2;
        boolean intakeOutButton = gamepad1.left_bumper;
        boolean footUpButton = gamepad1.dpad_up;
        boolean footDownButton = gamepad1.dpad_down;

        feetButton = footUpButton || footDownButton || isFootAllTheWayOut();

        if (intakeInButton) {
            intake.setPower(INTAKE_IN_POWER);
            if (!wasIntakeInPressed) {
                handleIntakeInJustPressed();
            }
        } else if (intakeOutButton) {
            intake.setPower(INTAKE_OUT_POWER);
            if (!wasIntakeOutPressed) {
                handleIntakeOutJustPressed();
            }
        } else {
            intake.setPower(INTAKE_OFF_POWER);
            if (isIntakeRunning || isIntakeReversing) {
                isIntakeRunning = false;
                isIntakeReversing = false;
            }
        }

        wasIntakeInPressed = intakeInButton;
        wasIntakeOutPressed = intakeOutButton;

        if (footUpButton && !wasFootUpPressed) {
            foot1.setPosition(FOOT_UP_POSITION);
            foot2.setPosition(FOOT_UP_POSITION);
        } else if (footDownButton && !wasFootDownPressed) {
            foot1.setPosition(FOOT_DOWN_POSITION);
            foot2.setPosition(FOOT_DOWN_POSITION);
        }

        wasFootUpPressed = footUpButton;
        wasFootDownPressed = footDownButton;

        pivotMode = CatapultModes.HOLD;
        catapult1.setPower(CATAPULT_HOLD_POWER);
        catapult2.setPower(CATAPULT_HOLD_POWER);
    }

    private void handleIntakeDuringLaunch() {
        boolean intakeInButton = gamepad1.left_trigger > 0.2;

        if (intakeInButton) {
            intake.setPower(INTAKE_IN_POWER);
        } else {
            intake.setPower(INTAKE_OFF_POWER);
        }
    }

    private void startVelocityShotController() {
        Pose currentPose = follower.getPose();

        // Calculate the heading to face the target
        double headingToTarget = velocityShotController.calculateHeading(
                currentPose.getX(),
                currentPose.getY()
        );

        Pose modifiedSetpoint = new Pose(
                VelocityShotSetpoint.getX(),
                VelocityShotSetpoint.getY(),
                headingToTarget
        );

        PathChain velocityShotPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, modifiedSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, modifiedSetpoint.getHeading(), 0.01))
                .build();

        follower.followPath(velocityShotPath);
        changeState(RobotState.NAVIGATING_TO_VELOCITY_SHOT);
    }

    private void handleNavigatingToVelocityShot() {
        if (checkForDriverInterruption()) return;

        double distanceToTarget = Math.hypot(
                follower.getPose().getX() - VelocityShotSetpoint.getX(),
                follower.getPose().getY() - VelocityShotSetpoint.getY()
        );

        if (distanceToTarget < velocityShotTolerance) {
            changeState(RobotState.LAUNCHING_UP);
        }
    }

    private void startNavigationToSetpoint() {
        PathChain setpointPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, targetSetpoint.getHeading(), 0.1))
                .build();

        follower.followPath(setpointPath);
        changeState(RobotState.NAVIGATING_TO_SETPOINT);
    }

    private void startNavigationToTargetOne() {
        PathChain targetOnePath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetOneSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, targetOneSetpoint.getHeading(), 0.1))
                .build();

        follower.followPath(targetOnePath);
        changeState(RobotState.NAVIGATING_TO_TARGET_ONE);
    }

    private void startNavigationToPark() {
        PathChain parkPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, parkPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, parkPose.getHeading(), 0.1))
                .build();

        follower.followPath(parkPath);
        changeState(RobotState.NAVIGATING_TO_PARK);
    }

    private void startNavigationToTargetTwo() {
        PathChain targetTwoPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetTwoSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, targetTwoSetpoint.getHeading(), 0.1))
                .build();

        follower.followPath(targetTwoPath);
        changeState(RobotState.NAVIGATING_TO_TARGET_TWO);
    }

    // Disabled — B is now heading lock.
    private void startNavigationToGate() {
        // PathChain gatePath = follower.pathBuilder()
        //         .addPath(new Path(new BezierLine(new Pose(follower.getPose().getX(),
        //                 follower.getPose().getY(), follower.getHeading()), gateWaypoint)))
        //         .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
        //                 follower::getHeading, gateWaypoint.getHeading(), 0.8))
        //         .addPath(new Path(new BezierLine(gateWaypoint, gateSetpoint)))
        //         .setHeadingInterpolation(HeadingInterpolator.linear(
        //                 gateWaypoint.getHeading(), gateSetpoint.getHeading()))
        //         .build();
        //
        // follower.followPath(gatePath);
        // changeState(RobotState.NAVIGATING_TO_GATE);
    }

    private void updateTelemetry() {
        String catapultModeStr;
        if (currentState == RobotState.STARTUP) {
            catapultModeStr = "STARTUP";
        } else if (isLaunchingState()) {
            catapultModeStr = currentState.toString();
        } else if (pivotMode == CatapultModes.UP) {
            catapultModeStr = "UP";
        } else if (pivotMode == CatapultModes.DOWN) {
            catapultModeStr = "DOWN";
        } else {
            catapultModeStr = "HOLD";
        }

        String activeTarget = getActiveTargetName();

        telemetry.addData("State", currentState.toString());
        telemetry.addData("Active Target", activeTarget);
        telemetry.addData("Catapult Mode", catapultModeStr);
        telemetry.addData("Foot1 Position", "%.3f", foot1.getPosition());
        telemetry.addData("Foot2 Position", "%.3f", foot2.getPosition());
        telemetry.addData("Distance to Goal", "%.2f", calculateDistanceToGoal());
        telemetry.addData("LED Mode", currentLedMode != null ? currentLedMode.toString() : "NONE");
        telemetry.addData("Velocity Shot Tolerance", "%.0f", velocityShotTolerance);

        // Live pose — copy these into addShotPose(...) when you find a spot that scores.
        Pose pose = follower.getPose();
        telemetry.addData("Pose X", "%.2f", pose.getX());
        telemetry.addData("Pose Y", "%.2f", pose.getY());
        telemetry.addData("Pose Heading (deg)", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Copy Shot Pose",
                "new Pose(%.4f, %.4f, Math.toRadians(%.2f))",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));

        // HeatMap telemetry
        telemetry.addData("HeatMap Active", heatMapController.isActive() ? "YES" : "NO");
        telemetry.addData("HeatMap Shot Ready", heatMapController.isShotReady() ? "YES" : "NO");
        telemetry.addData("Nearest Shot Index", heatMapController.getClosestShotIndex());
        telemetry.addData("Blending Segment",
                "A=%d  B=%d  t=%.2f",
                heatMapController.getSegmentAIndex(),
                heatMapController.getSegmentBIndex(),
                heatMapController.getSegmentT());
        Pose closest = heatMapController.getClosestShotPose();
        if (closest != null) {
            telemetry.addData("Effective Shot Pose",
                    "X:%.2f Y:%.2f H:%.2f",
                    closest.getX(), closest.getY(), Math.toDegrees(closest.getHeading()));
        }
        telemetry.addData("Dist to Segment", "%.2f", heatMapController.getClosestDistance());
        telemetry.addData("Heading Error (deg)", "%.2f", heatMapController.getHeadingErrorDeg());
        telemetry.addData("Pos OK / Head OK",
                "%s / %s",
                heatMapController.isPositionOk() ? "YES" : "no",
                heatMapController.isHeadingOk() ? "YES" : "no");
        telemetry.addData("HeatMap RGB", "R:%d G:%d B:%d",
                heatMapController.getCurrentR(),
                heatMapController.getCurrentG(),
                heatMapController.getCurrentB());

        telemetry.update();
    }

    private boolean isLaunchingState() {
        return currentState == RobotState.LAUNCHING_UP ||
                currentState == RobotState.LAUNCHING_DOWN ||
                currentState == RobotState.LAUNCHING_HOLD;
    }

    private String getActiveTargetName() {
        switch (currentState) {
            case VELOCITY_SHOT_ACTIVE:
                return "VELOCITY_SHOT_CONTROLLER";
            case NAVIGATING_TO_SETPOINT:
                return "ORIGINAL";
            case NAVIGATING_TO_TARGET_ONE:
                return "TARGET_ONE";
            case NAVIGATING_TO_TARGET_TWO:
                return "TARGET_TWO";
            case NAVIGATING_TO_GATE:
                return "GATE";
            case NAVIGATING_TO_VELOCITY_SHOT:
            case NAVIGATING_TO_PARK:
                return "PARK";
            case NAVIGATING_TO_VELOCITY_SCORE:
                return "VELOCITY_SHOT";
            default:
                return "NONE";
        }
    }
}