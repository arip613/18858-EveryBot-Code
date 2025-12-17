package org.firstinspires.ftc.teamcode.Teleop;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "BLUE TELEOP", group = "Teleop")
public class BLUE_TELEOP extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();
    private ElapsedTime autoDownTimer = new ElapsedTime();
    private ElapsedTime setpointLaunchTimer = new ElapsedTime();
    private ElapsedTime parkFootTimer = new ElapsedTime();
    private ElapsedTime startupCatapultTimer = new ElapsedTime();

    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private boolean setpointNavActive = false;
    private boolean setpointReached = false;
    private Pose targetSetpoint = new Pose(114.585, 123.805, Math.toRadians(38)).mirror();

    private boolean targetOneNavActive = false;
    private boolean targetOneReached = false;
    private Pose targetOneSetpoint = new Pose(95.26829268292684, 104.04878048780488, Math.toRadians(37)).mirror();

    private boolean targetTwoNavActive = false;
    private boolean targetTwoReached = false;
    private Pose targetTwoSetpoint = new Pose(109.09756097560975, 112.60975609756098, Math.toRadians(37)).mirror();

    // Gate setpoint (B button)
    private boolean gateNavActive = false;
    private boolean gateReached = false;
    private Pose gateSetpoint = new Pose(128.657, 69.041, Math.toRadians(180)).mirror();

    private Pose gateWaypoint = new Pose(120.73170731707316, 71.34146341463415, Math.toRadians(180)).mirror();


    private static final double SETPOINT_TOLERANCE = 2; // inches

    // Park setpoint variables
    private boolean parkNavActive = false;
    private boolean parkReached = false;
    private Pose parkSetpoint = new Pose(45.151, 40.329, Math.toRadians(225)).mirror();

    private boolean parkFootActive = false;
    private static final double PARK_FOOT_DURATION = 2;

    private enum LaunchState {IDLE, LAUNCHING_UP, LAUNCHING_DOWN, LAUNCHING_HOLD}
    private LaunchState launchState = LaunchState.IDLE;
    private static final double LAUNCH_UP_DURATION = 0.1;
    private static final double LAUNCH_DOWN_DURATION = 0.25;

    private boolean startupCatapultActive = true;
    private static final double STARTUP_DOWN_DURATION = 0.15;

    private DcMotor intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private DcMotor foot = null;

    // Intake power constants
    private double INTAKE_IN_POWER = -1;
    private double INTAKE_OUT_POWER = 1;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    private double FOOT_UP_POWER = -1.0;
    private double FOOT_DOWN_POWER = 0.85;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;

    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1;
    private double CATAPULT_HOLD_POWER = 0.2;

    private boolean autoDownActive = false;
    private boolean wasUpButtonPressed = false;
    private static final double AUTO_DOWN_DURATION = 0.25;

    private enum CatapultModes {UP, DOWN, HOLD}
    private CatapultModes pivotMode;

    private enum FootMode {UP, DOWN, BRAKE}
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
        foot = hardwareMap.get(DcMotor.class, "foot");

        intake.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        foot.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        foot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        runtime.reset();
        catatime.reset();
        startupCatapultTimer.reset();
        startupCatapultActive = true;
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // Handle startup catapult sequence
        if (startupCatapultActive) {
            if (startupCatapultTimer.seconds() < STARTUP_DOWN_DURATION) {
                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);
                pivotMode = CatapultModes.DOWN;
            } else {
                catapult1.setPower(CATAPULT_HOLD_POWER);
                catapult2.setPower(CATAPULT_HOLD_POWER);
                pivotMode = CatapultModes.HOLD;
                startupCatapultActive = false;
            }
        }

        if (gamepad1.back) {
            Pose currentPose = follower.getPose();
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), 180));
        }

        boolean driverInput = Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1;

        boolean anyLaunchNavActive = setpointNavActive || targetOneNavActive || targetTwoNavActive || gateNavActive;

        // Original launch setpoint navigation (Y button)
        if (gamepad1.y && !anyLaunchNavActive && !parkNavActive) {
            PathChain setpointPath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, targetSetpoint)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            follower::getHeading,
                            targetSetpoint.getHeading(),
                            0.8))
                    .build();

            follower.followPath(setpointPath);
            setpointNavActive = true;
            setpointReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = true;
        }

        // Target one setpoint navigation (A button)
        if (gamepad1.a && !anyLaunchNavActive && !parkNavActive) {
            PathChain targetOnePath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, targetOneSetpoint)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            follower::getHeading,
                            targetOneSetpoint.getHeading(),
                            0.8))
                    .build();

            follower.followPath(targetOnePath);
            targetOneNavActive = true;
            targetOneReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = true;
        }

        // Target two setpoint navigation (X button)
        if (gamepad1.x && !anyLaunchNavActive && !parkNavActive) {
            PathChain targetTwoPath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, targetTwoSetpoint)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            follower::getHeading,
                            targetTwoSetpoint.getHeading(),
                            0.8))
                    .build();

            follower.followPath(targetTwoPath);
            targetTwoNavActive = true;
            targetTwoReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = true;
        }

        // Gate setpoint navigation (B button)
        if (gamepad1.b && !anyLaunchNavActive && !parkNavActive) {
            PathChain gatePath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading()), gateWaypoint)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            follower::getHeading, gateWaypoint.getHeading(), 0.8))
                    .addPath(new Path(new BezierLine(gateWaypoint, gateSetpoint)))
                    .setHeadingInterpolation(HeadingInterpolator.linear(
                            gateWaypoint.getHeading(), gateSetpoint.getHeading()))
                    .build();

            follower.followPath(gatePath);
            gateNavActive = true;
            gateReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = true;
        }

        // Park setpoint navigation (D-pad left or right)
        if ((gamepad1.dpad_left || gamepad1.dpad_right) && !parkNavActive && !anyLaunchNavActive) {
            PathChain parkPath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, parkSetpoint)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            follower::getHeading,
                            parkSetpoint.getHeading(),
                            0.8))
                    .build();

            follower.followPath(parkPath);
            parkNavActive = true;
            parkReached = false;
            parkFootActive = false;
            automatedDrive = true;
        }

        // Cancel original setpoint navigation on driver input
        if (setpointNavActive && driverInput) {
            follower.startTeleopDrive();
            setpointNavActive = false;
            setpointReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = false;
        }

        // Cancel target one navigation on driver input
        if (targetOneNavActive && driverInput) {
            follower.startTeleopDrive();
            targetOneNavActive = false;
            targetOneReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = false;
        }

        // Cancel target two navigation on driver input
        if (targetTwoNavActive && driverInput) {
            follower.startTeleopDrive();
            targetTwoNavActive = false;
            targetTwoReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = false;
        }

        // Cancel gate navigation on driver input
        if (gateNavActive && driverInput) {
            follower.startTeleopDrive();
            gateNavActive = false;
            gateReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = false;
        }

        // Cancel park navigation on driver input
        if (parkNavActive && driverInput) {
            follower.startTeleopDrive();
            parkNavActive = false;
            parkReached = false;
            parkFootActive = false;
            automatedDrive = false;
        }

        // Check if original launch setpoint reached
        if (setpointNavActive && !setpointReached) {
            double distanceToTarget = Math.hypot(
                    follower.getPose().getX() - targetSetpoint.getX(),
                    follower.getPose().getY() - targetSetpoint.getY()
            );

            if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
                setpointReached = true;
                launchState = LaunchState.LAUNCHING_UP;
                setpointLaunchTimer.reset();
            }
        }

        // Check if target one setpoint reached
        if (targetOneNavActive && !targetOneReached) {
            double distanceToTarget = Math.hypot(
                    follower.getPose().getX() - targetOneSetpoint.getX(),
                    follower.getPose().getY() - targetOneSetpoint.getY()
            );

            if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
                targetOneReached = true;
                launchState = LaunchState.LAUNCHING_UP;
                setpointLaunchTimer.reset();
            }
        }

        // Check if target two setpoint reached
        if (targetTwoNavActive && !targetTwoReached) {
            double distanceToTarget = Math.hypot(
                    follower.getPose().getX() - targetTwoSetpoint.getX(),
                    follower.getPose().getY() - targetTwoSetpoint.getY()
            );

            if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
                targetTwoReached = true;
                launchState = LaunchState.LAUNCHING_UP;
                setpointLaunchTimer.reset();
            }
        }

        // Check if gate setpoint reached
        if (gateNavActive && !gateReached) {
            double distanceToTarget = Math.hypot(
                    follower.getPose().getX() - gateSetpoint.getX(),
                    follower.getPose().getY() - gateSetpoint.getY()
            );

            if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
                gateReached = true;
                launchState = LaunchState.LAUNCHING_UP;
                setpointLaunchTimer.reset();
            }
        }

        // Check if park setpoint reached
        if (parkNavActive && !parkReached) {
            double distanceToPark = Math.hypot(
                    follower.getPose().getX() - parkSetpoint.getX(),
                    follower.getPose().getY() - parkSetpoint.getY()
            );

            if (distanceToPark < SETPOINT_TOLERANCE && !follower.isBusy()) {
                parkReached = true;
                parkFootActive = true;
                parkFootTimer.reset();
            }
        }

        // Handle launch sequence when any setpoint reached
        boolean anySetpointReached = setpointReached || targetOneReached || targetTwoReached || gateReached;
        if (anySetpointReached) {
            switch (launchState) {
                case LAUNCHING_UP:
                    catapult1.setPower(CATAPULT_UP_POWER);
                    catapult2.setPower(CATAPULT_UP_POWER);
                    if (setpointLaunchTimer.seconds() >= LAUNCH_UP_DURATION) {
                        launchState = LaunchState.LAUNCHING_DOWN;
                        setpointLaunchTimer.reset();
                    }
                    break;

                case LAUNCHING_DOWN:
                    catapult1.setPower(CATAPULT_DOWN_POWER);
                    catapult2.setPower(CATAPULT_DOWN_POWER);
                    if (setpointLaunchTimer.seconds() >= LAUNCH_DOWN_DURATION) {
                        launchState = LaunchState.LAUNCHING_HOLD;
                        // Reset all launch navigation states
                        setpointNavActive = false;
                        setpointReached = false;
                        targetOneNavActive = false;
                        targetOneReached = false;
                        targetTwoNavActive = false;
                        targetTwoReached = false;
                        gateNavActive = false;
                        gateReached = false;
                        automatedDrive = false;
                        follower.startTeleopDrive();
                    }
                    break;

                case LAUNCHING_HOLD:
                    catapult1.setPower(CATAPULT_HOLD_POWER);
                    catapult2.setPower(CATAPULT_HOLD_POWER);
                    break;

                case IDLE:
                    break;
            }
        }

        if (parkReached && parkFootActive) {
                parkNavActive = false;
                parkReached = false;
                automatedDrive = false;
                follower.startTeleopDrive();
        }

        // Manual drive control
        if (!automatedDrive && !anyLaunchNavActive && !parkNavActive) {

            follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // false = field centric
            );
        }

        // Cancel automated drive
        if (automatedDrive && !anyLaunchNavActive && !parkNavActive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.left_bumper) {
            slowMode = !slowMode;
        }

        // Manual control (only when not in automated sequences and startup is complete)
        if (!anySetpointReached && !parkReached && !startupCatapultActive) {
            boolean intakeInButton = gamepad1.left_trigger > 0.2;
            boolean intakeOutButton = gamepad1.left_bumper;
            boolean catapultUpButton = gamepad1.right_bumper;
            boolean catapultDownButton = gamepad1.right_trigger > 0.2;
            boolean footUpButton = gamepad1.dpad_up;
            boolean footDownButton = gamepad1.dpad_down;

            if (wasUpButtonPressed && !catapultUpButton && !autoDownActive) {
                autoDownActive = true;
                autoDownTimer.reset();
            }

            wasUpButtonPressed = catapultUpButton;

            if (autoDownActive && autoDownTimer.seconds() >= AUTO_DOWN_DURATION) {
                autoDownActive = false;
            }

            if (catapultUpButton && catapultDownButton) {
                catapultUpButton = false;
            }

            if (intakeInButton) {
                intakePower = INTAKE_IN_POWER;
            } else if (intakeOutButton) {
                intakePower = INTAKE_OUT_POWER;
            } else {
                intakePower = INTAKE_OFF_POWER;
            }

            if (footUpButton) {
                footmode = FootMode.UP;
                footPower = FOOT_UP_POWER;
            } else if (footDownButton) {
                footmode = FootMode.DOWN;
                footPower = FOOT_DOWN_POWER;
            } else {
                footmode = FootMode.BRAKE;
                footPower = FOOT_OFF_POWER;
            }

            if (catapultUpButton) {
                pivotMode = CatapultModes.UP;
                catapult1.setPower(CATAPULT_UP_POWER);
                catapult2.setPower(CATAPULT_UP_POWER);
            } else if (catapultDownButton || autoDownActive) {
                pivotMode = CatapultModes.DOWN;
                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);
            } else {
                pivotMode = CatapultModes.HOLD;
                catapult1.setPower(CATAPULT_HOLD_POWER);
                catapult2.setPower(CATAPULT_HOLD_POWER);
            }

            intake.setPower(intakePower);
            if (!parkFootActive) {
                foot.setPower(footPower);
            }
        } else if (anySetpointReached) {
            boolean intakeInButton = gamepad1.left_trigger > 0.2;
            boolean footOutButton = gamepad1.a;

            if (intakeInButton) {
                intakePower = INTAKE_IN_POWER;
            } else {
                intakePower = INTAKE_OFF_POWER;
            }

            if (footOutButton) {
                footmode = FootMode.DOWN;
                footPower = FOOT_DOWN_POWER;
            } else {
                footmode = FootMode.BRAKE;
                footPower = FOOT_OFF_POWER;
            }

            intake.setPower(intakePower);
            foot.setPower(footPower);
        }

        String catapult_mode_str;
        if (startupCatapultActive) {
            catapult_mode_str = "STARTUP";
        } else if (anySetpointReached) {
            catapult_mode_str = launchState.toString();
        } else if (pivotMode == CatapultModes.UP) {
            catapult_mode_str = "UP";
        } else if (pivotMode == CatapultModes.DOWN) {
            catapult_mode_str = "DOWN";
        } else {
            catapult_mode_str = "HOLD";
        }

        String activeTarget = "NONE";
        if (setpointNavActive) activeTarget = "ORIGINAL";
        if (targetOneNavActive) activeTarget = "TARGET_ONE";
        if (targetTwoNavActive) activeTarget = "TARGET_TWO";
        if (gateNavActive) activeTarget = "GATE";
        if (parkNavActive) activeTarget = "PARK";

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Startup Catapult Active", startupCatapultActive);
        telemetry.addData("Active Target", activeTarget);
        telemetry.addData("Setpoint Reached", setpointReached);
        telemetry.addData("Target One Reached", targetOneReached);
        telemetry.addData("Target Two Reached", targetTwoReached);
        telemetry.addData("Gate Reached", gateReached);
        telemetry.addData("Park Reached", parkReached);
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Automated Drive", automatedDrive);
        telemetry.addData("Auto Down Active", autoDownActive);
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Foot Power/Mode", "%.2f, %s", foot.getPower(), footmode);
        telemetry.addData("Catapult1 Pos/Power", "%d, %.2f",
                catapult1.getCurrentPosition(), catapult1.getPower());
        telemetry.addData("Catapult2 Pos/Power", "%d, %.2f",
                catapult2.getCurrentPosition(), catapult2.getPower());
        telemetry.addData("Catapult Mode", catapult_mode_str);
        telemetry.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("activeTarget", activeTarget);
    }
}