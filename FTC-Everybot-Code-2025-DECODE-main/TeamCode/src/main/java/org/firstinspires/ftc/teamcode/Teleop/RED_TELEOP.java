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
@TeleOp(name = "RED TELEOP", group = "Teleop")
public class RED_TELEOP extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();
    private ElapsedTime autoDownTimer = new ElapsedTime();
    private ElapsedTime setpointLaunchTimer = new ElapsedTime();

    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private boolean setpointNavActive = false;
    private boolean setpointReached = false;
    private Pose targetSetpoint = new Pose(118.537, 119.634, Math.toRadians(37));
    private static final double SETPOINT_TOLERANCE = 2.0; // inches

    private enum LaunchState {IDLE, LAUNCHING_UP, LAUNCHING_DOWN, LAUNCHING_HOLD}
    private LaunchState launchState = LaunchState.IDLE;
    private static final double LAUNCH_UP_DURATION = 0.5;
    private static final double LAUNCH_DOWN_DURATION = 0.15;

    // Subsystem motors
    private DcMotor intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private DcMotor foot = null;

    // Intake power constants
    private double INTAKE_IN_POWER = -1;
    private double INTAKE_OUT_POWER = -1;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    private double FOOT_UP_POWER = 1.0;
    private double FOOT_DOWN_POWER = -0.85;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;

    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1;
    private double CATAPULT_HOLD_POWER = 0.2;

    private boolean autoDownActive = false;
    private boolean wasUpButtonPressed = false;
    private static final double AUTO_DOWN_DURATION = 0.15;

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

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

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
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        boolean driverInput = Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1;

        if (gamepad1.y && !setpointNavActive) {
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

        if (setpointNavActive && driverInput) {
            follower.startTeleopDrive();
            setpointNavActive = false;
            setpointReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = false;
        }

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

        if (setpointReached) {
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
                        setpointNavActive = false;
                        setpointReached = false;
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

        if (!automatedDrive && !setpointNavActive) {
            if (!slowMode) {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // false = field centric
                );
            } else {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y * slowModeMultiplier,
                        gamepad1.left_stick_x * slowModeMultiplier,
                        gamepad1.right_stick_x * slowModeMultiplier,
                        false
                );
            }
        }

        if (gamepad1.dpad_up && !setpointNavActive) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && !setpointNavActive && (gamepad1.dpad_down || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.left_bumper) {
            slowMode = !slowMode;
        }

        if (!setpointReached) {
            boolean intakeInButton = gamepad1.left_trigger > 0.2;

            boolean footOutButton = gamepad1.a;
            boolean footUpButton = gamepad1.b;

            if (footOutButton && footUpButton) {
                footOutButton = false;
            }

            boolean catapultUpButton = gamepad1.right_bumper;
            boolean catapultDownButton = gamepad1.right_trigger > 0.2;


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
            foot.setPower(footPower);
        } else {
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
        if (setpointReached) {
            catapult_mode_str = launchState.toString();
        } else if (pivotMode == CatapultModes.UP) {
            catapult_mode_str = "UP";
        } else if (pivotMode == CatapultModes.DOWN) {
            catapult_mode_str = "DOWN";
        } else {
            catapult_mode_str = "HOLD";
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Setpoint Nav Active", setpointNavActive);
        telemetry.addData("Setpoint Reached", setpointReached);
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
        telemetryM.debug("setpointNavActive", setpointNavActive);
    }
}