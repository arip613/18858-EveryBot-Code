package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
    private Pose VelocityShotSetpoint = new Pose(134, 134, Math.toRadians(37)).mirror();
    private Pose targetSetpoint = new Pose(121.40906836664445, 122.0041889556019, Math.toRadians(40)).mirror();
    private Pose targetOneSetpoint = new Pose(102.34525660964229, 110.63141524105754, Math.toRadians(37)).mirror();
    private Pose targetTwoSetpoint = new Pose(112.64696734059099, 119.58942457231727, Math.toRadians(37)).mirror();
    private Pose gateSetpoint = new Pose(128.657, 72, Math.toRadians(90)).mirror();
    private Pose gateWaypoint = new Pose(120.73170731707316, 72.5, Math.toRadians(90)).mirror();
    private Pose parkPose = new Pose(43.474387184684886, 39.59050310801249, Math.toRadians(230)).mirror();

    // ==================== CONSTANTS ====================
    private static final double SETPOINT_TOLERANCE = 2;
    private static final double LAUNCH_UP_DURATION = 0.2;
    private static final double LAUNCH_DOWN_DURATION = 0.4;
    private static final double STARTUP_DOWN_DURATION = 0.25;

    private double INTAKE_IN_POWER = -1.0;
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

    // ==================== LED VARIABLES ====================
    private GoBildaPrismDriver prism = null;
    private PrismAnimations.Solid solidRed = new PrismAnimations.Solid(Color.RED);
    private PrismAnimations.Solid solidBlue = new PrismAnimations.Solid(Color.BLUE);
    private PrismAnimations.Solid solidWhite = new PrismAnimations.Solid(Color.WHITE);
    private PrismAnimations.Solid solidCyan = new PrismAnimations.Solid(Color.CYAN);
    private PrismAnimations.Blink blinkBlueWhite = new PrismAnimations.Blink(Color.BLUE, Color.WHITE);
    private PrismAnimations.Snakes swirlAnimation = new PrismAnimations.Snakes(Color.BLUE, Color.CYAN, Color.WHITE);

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

        // Initialize HeatMap controller
        heatMapController = new HeatMapController(prism);

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

        Pose3D botpose3D = result.getBotpose();
        Pose currentPose = follower.getPose();

        if (result != null && result.isValid()
                && botpose3D != null
                && result.getBotposeTagCount() > 0) {

            Vector velocity = follower.getVelocity();
            double angularVelocity = follower.getAngularVelocity();

            double linearVelocity = Math.hypot(
                    velocity.getXComponent(),
                    velocity.getYComponent()
            );

            if (linearVelocity < 2 && Math.abs(angularVelocity) < 2) {

                Pose limelightPose = LimelightPoseUpdater.convertLimelightToPedro(
                        botpose3D.getPosition().x,
                        botpose3D.getPosition().y,
                        currentPose.getHeading()
                );

                follower.setPose(limelightPose);
            }
        }

        // Update LEDs
        // updateLEDs();

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
            case NAVIGATING_TO_GATE:
                handleNavigatingToGate();
                break;
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
/*
    // ==================== LED UPDATE SYSTEM ====================

    private void updateLEDs() {
        // First check if HeatMap should be active
        if (isNavigatingState()) {
            Pose targetPose = getActiveGoalPose();
            boolean heatmapActive = heatMapController.update(follower, targetPose);

            if (heatmapActive) {
                currentLedMode = LedMode.HEATMAP;
                return; // HeatMap is handling LEDs, don't override
            }
        } else {
            // Reset heatmap when not navigating
            heatMapController.reset();
        }

        // Fall back to normal LED behavior
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
                setLedMode(LedMode.SOLID_BLUE);
                break;

            default:
                setLedMode(LedMode.BLINK);
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
            case HEATMAP:
                // Handled by HeatMapController
                break;
        }
    }
    */


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
            velocityShotTolerance = 50;
            startVelocityShotController();
            return;
        }

        // Right Bumper + X = velocity shot with tolerance 80
        if (gamepad1.right_bumper && gamepad1.x) {
            velocityShotTolerance = 80;
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
        if (gamepad1.b) {
            startNavigationToGate();
            return;
        }

        follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        handleManualControls();
    }

    private void handleNavigatingToSetpoint() {
        if (checkForDriverInterruption()) return;

        double distanceToTarget = Math.hypot(
                follower.getPose().getX() - targetSetpoint.getX(),
                follower.getPose().getY() - targetSetpoint.getY()
        );

        if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
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

        double distanceToTarget = Math.hypot(
                follower.getPose().getX() - targetOneSetpoint.getX(),
                follower.getPose().getY() - targetOneSetpoint.getY()
        );

        if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
            changeState(RobotState.LAUNCHING_UP);
        }
    }

    private void handleNavigatingToTargetTwo() {
        if (checkForDriverInterruption()) return;

        double distanceToTarget = Math.hypot(
                follower.getPose().getX() - targetTwoSetpoint.getX(),
                follower.getPose().getY() - targetTwoSetpoint.getY()
        );

        if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
            changeState(RobotState.LAUNCHING_UP);
        }
    }

    private void handleNavigatingToGate() {
        if (checkForDriverInterruption()) return;

        double distanceToTarget = Math.hypot(
                follower.getPose().getX() - gateSetpoint.getX(),
                follower.getPose().getY() - gateSetpoint.getY()
        );

        if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
            changeState(RobotState.MANUAL_DRIVE);
        }
    }

    private void handleLaunchingUp() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);

        handleIntakeDuringLaunch();

        if (stateTimer.seconds() >= LAUNCH_UP_DURATION) {
            changeState(RobotState.LAUNCHING_DOWN);
        }
    }

    private void handleLaunchingDown() {
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);

        handleIntakeDuringLaunch();

        if (stateTimer.seconds() >= LAUNCH_DOWN_DURATION) {
            changeState(RobotState.LAUNCHING_HOLD);
        }
    }

    private void handleLaunchingHold() {
        catapult1.setPower(CATAPULT_HOLD_POWER);
        catapult2.setPower(CATAPULT_HOLD_POWER);
        follower.startTeleopDrive();
        changeState(RobotState.MANUAL_DRIVE);
    }

    // ==================== HELPER METHODS ====================

    private void changeState(RobotState newState) {
        previousState = currentState;
        currentState = newState;
        stateTimer.reset();
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

    private void startNavigationToGate() {
        PathChain gatePath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Pose(follower.getPose().getX(),
                        follower.getPose().getY(), follower.getHeading()), gateWaypoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, gateWaypoint.getHeading(), 0.8))
                .addPath(new Path(new BezierLine(gateWaypoint, gateSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linear(
                        gateWaypoint.getHeading(), gateSetpoint.getHeading()))
                .build();

        follower.followPath(gatePath);
        changeState(RobotState.NAVIGATING_TO_GATE);
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

        // Add HeatMap telemetry
        telemetry.addData("HeatMap Active", heatMapController.isActive() ? "YES" : "NO");
        telemetry.addData("HeatMap RGB", String.format("R:%d G:%d B:%d",
                heatMapController.getCurrentR(),
                heatMapController.getCurrentG(),
                heatMapController.getCurrentB()));

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