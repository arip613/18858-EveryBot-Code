package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "VSHOT_SM", group = "Teleop")
public class STATEMACHINEPERCHANCE extends OpMode {

    // ==================== STATE MACHINE ENUMS ====================
    private enum RobotState {
        STARTUP,
        MANUAL_DRIVE,
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

    // ==================== NAVIGATION SETPOINTS ====================
    public static Pose startingPose;
    private Pose VelocityShotSetpoint = new Pose(72, 72, Math.toRadians(37));
    private Pose ScoreSetpoint = new Pose(112.6750092686662, 122.45208942216487, Math.toRadians(37));
    private Pose targetSetpoint = new Pose(112.6750092686662, 122.45208942216487, Math.toRadians(37));
    private Pose targetOneSetpoint = new Pose(102.34525660964229, 110.63141524105754, Math.toRadians(37));
    private Pose targetTwoSetpoint = new Pose(112.64696734059099, 119.58942457231727, Math.toRadians(37));
    private Pose gateSetpoint = new Pose(128.657, 72, Math.toRadians(90));
    private Pose gateWaypoint = new Pose(120.73170731707316, 72.5, Math.toRadians(90));

    // ==================== CONSTANTS ====================
    private static final double SETPOINT_TOLERANCE = 2.0;
    private static final double VELOCITY_SHOT_TOLERANCE = 24.0;
    private static final double LAUNCH_UP_DURATION = 0.1;
    private static final double LAUNCH_DOWN_DURATION = 0.4;
    private static final double STARTUP_DOWN_DURATION = 0.25;
    private static final double PARK_FOOT_DURATION = 2.0;

    private double INTAKE_IN_POWER = -1.0;
    private double INTAKE_OUT_POWER = 1.0;
    private double INTAKE_OFF_POWER = 0.0;
    private double FOOT_UP_POSITION = 0.85;
    private double FOOT_DOWN_POSITION = 0.525;
    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
    private double CATAPULT_HOLD_POWER = 0.0;

    // ==================== CONTROL VARIABLES ====================
    private boolean slowMode = false;
    private boolean wasFootUpPressed = false;
    private boolean wasFootDownPressed = false;
    private CatapultModes pivotMode;
    private FootMode footmode;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        runtime.reset();
        stateTimer.reset();
        changeState(RobotState.STARTUP);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // Handle input that applies to all states
        handleGlobalInput();

        // Execute current state
        switch (currentState) {
            case STARTUP:
                handleStartup();
                break;
            case MANUAL_DRIVE:
                handleManualDrive();
                break;
            case NAVIGATING_TO_VELOCITY_SHOT:
                handleNavigatingToVelocityShot();
                break;
            case NAVIGATING_TO_VELOCITY_SCORE:
                handleNavigatingToVelocityScore();
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
        // Check for navigation requests
        if (gamepad1.right_bumper) {
            startVelocityShotSequence();
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

        // Manual drive control
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        // Handle manual controls
        handleManualControls();
    }

    private void handleNavigatingToVelocityShot() {
        if (checkForDriverInterruption()) return;

        double distanceToTarget = Math.hypot(
                follower.getPose().getX() - VelocityShotSetpoint.getX(),
                follower.getPose().getY() - VelocityShotSetpoint.getY()
        );

        if (distanceToTarget < SETPOINT_TOLERANCE) {
            changeState(RobotState.NAVIGATING_TO_VELOCITY_SCORE);
        }
    }

    private void handleNavigatingToVelocityScore() {
        if (checkForDriverInterruption()) return;

        double distanceToTarget = Math.hypot(
                follower.getPose().getX() - ScoreSetpoint.getX(),
                follower.getPose().getY() - ScoreSetpoint.getY()
        );

        if (distanceToTarget < VELOCITY_SHOT_TOLERANCE) {
            changeState(RobotState.LAUNCHING_UP);
        }
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

        // Allow intake during launch
        handleIntakeDuringLaunch();

        if (stateTimer.seconds() >= LAUNCH_UP_DURATION) {
            changeState(RobotState.LAUNCHING_DOWN);
        }
    }

    private void handleLaunchingDown() {
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);

        // Allow intake during launch
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
        // Reset heading with back button
        if (gamepad1.back) {
            Pose currentPose = follower.getPose();
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), 180));
        }

        // Reset heading with start button
        if (gamepad1.start) {
            Pose currentPose = follower.getPose();
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), 0));
        }

        // Toggle slow mode
        if (gamepad1.left_bumper) {
            slowMode = !slowMode;
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

    private void handleManualControls() {
        boolean intakeInButton = gamepad1.left_trigger > 0.2;
        boolean intakeOutButton = gamepad1.left_bumper;
        boolean catapultDownButton = gamepad1.right_trigger > 0.2;
        boolean footUpButton = gamepad1.dpad_up;
        boolean footDownButton = gamepad1.dpad_down;

        // Intake control
        if (intakeInButton) {
            intake.setPower(INTAKE_IN_POWER);
        } else if (intakeOutButton) {
            intake.setPower(INTAKE_OUT_POWER);
        } else {
            intake.setPower(INTAKE_OFF_POWER);
        }

        // Foot control
        if (footUpButton && !wasFootUpPressed) {
            foot1.setPosition(FOOT_UP_POSITION);
            foot2.setPosition(FOOT_UP_POSITION);
        } else if (footDownButton && !wasFootDownPressed) {
            foot1.setPosition(FOOT_DOWN_POSITION);
            foot2.setPosition(FOOT_DOWN_POSITION);
        }

        wasFootUpPressed = footUpButton;
        wasFootDownPressed = footDownButton;

        // Catapult control
        if (catapultDownButton) {
            pivotMode = CatapultModes.DOWN;
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
        } else {
            pivotMode = CatapultModes.HOLD;
            catapult1.setPower(CATAPULT_HOLD_POWER);
            catapult2.setPower(CATAPULT_HOLD_POWER);
        }
    }

    private void handleIntakeDuringLaunch() {
        boolean intakeInButton = gamepad1.left_trigger > 0.2;

        if (intakeInButton) {
            intake.setPower(INTAKE_IN_POWER);
        } else {
            intake.setPower(INTAKE_OFF_POWER);
        }
    }

    private void startVelocityShotSequence() {
        PathChain velocityShotPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, VelocityShotSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, VelocityShotSetpoint.getHeading(), 0.8))
                .addPath(new Path(new BezierLine(VelocityShotSetpoint, ScoreSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linear(
                        VelocityShotSetpoint.getHeading(), ScoreSetpoint.getHeading()))
                .build();

        follower.followPath(velocityShotPath);
        changeState(RobotState.NAVIGATING_TO_VELOCITY_SHOT);
    }

    private void startNavigationToSetpoint() {
        PathChain setpointPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, targetSetpoint.getHeading(), 0.8))
                .build();

        follower.followPath(setpointPath);
        changeState(RobotState.NAVIGATING_TO_SETPOINT);
    }

    private void startNavigationToTargetOne() {
        PathChain targetOnePath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetOneSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, targetOneSetpoint.getHeading(), 0.8))
                .build();

        follower.followPath(targetOnePath);
        changeState(RobotState.NAVIGATING_TO_TARGET_ONE);
    }

    private void startNavigationToTargetTwo() {
        PathChain targetTwoPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetTwoSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, targetTwoSetpoint.getHeading(), 0.8))
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
        telemetry.update();
    }

    private boolean isLaunchingState() {
        return currentState == RobotState.LAUNCHING_UP ||
                currentState == RobotState.LAUNCHING_DOWN ||
                currentState == RobotState.LAUNCHING_HOLD;
    }

    private String getActiveTargetName() {
        switch (currentState) {
            case NAVIGATING_TO_SETPOINT:
                return "ORIGINAL";
            case NAVIGATING_TO_TARGET_ONE:
                return "TARGET_ONE";
            case NAVIGATING_TO_TARGET_TWO:
                return "TARGET_TWO";
            case NAVIGATING_TO_GATE:
                return "GATE";
            case NAVIGATING_TO_VELOCITY_SHOT:
            case NAVIGATING_TO_VELOCITY_SCORE:
                return "VELOCITY_SHOT";
            default:
                return "NONE";
        }
    }
}