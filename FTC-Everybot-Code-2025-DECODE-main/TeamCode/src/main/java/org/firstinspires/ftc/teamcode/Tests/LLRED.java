package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "LIMELIGHT WITH RED", group = "Teleop")
public class LLRED extends OpMode {

    // Kalman Filter
    private static class KalmanFilter {
        private double processNoise;
        private double measurementNoise;
        private double estimation;
        private double errorCovariance;

        public KalmanFilter(double processNoise, double measurementNoise, double initialEstimate) {
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
            this.estimation = initialEstimate;
            this.errorCovariance = 1;
        }

        public double update(double measurement, double prediction) {
            // Prediction step
            double predictedEstimate = prediction;
            double predictedCovariance = errorCovariance + processNoise;

            // Update step
            double kalmanGain = predictedCovariance / (predictedCovariance + measurementNoise);
            estimation = predictedEstimate + kalmanGain * (measurement - predictedEstimate);
            errorCovariance = (1 - kalmanGain) * predictedCovariance;

            return estimation;
        }

        public void reset(double value) {
            this.estimation = value;
            this.errorCovariance = 1;
        }
    }

    private KalmanFilter xFilter;
    private KalmanFilter yFilter;
    private KalmanFilter headingFilter;

    private ElapsedTime runtime = new ElapsedTime();
    private Limelight3A camera;
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

    // Limelight odometry variables
    private boolean useLimelightOdometry = true;

    private boolean setpointNavActive = false;
    private boolean setpointReached = false;
    private Pose targetSetpoint = new Pose(118.537, 119.634, Math.toRadians(37));

    private boolean targetOneNavActive = false;
    private boolean targetOneReached = false;
    private Pose targetOneSetpoint = new Pose(97.6829268292683, 104.04878048780488, Math.toRadians(37));

    private boolean targetTwoNavActive = false;
    private boolean targetTwoReached = false;
    private Pose targetTwoSetpoint = new Pose(112.390, 114.585, Math.toRadians(37));

    private boolean gateNavActive = false;
    private boolean gateReached = false;
    private Pose gateSetpoint = new Pose(128.657, 69.041, Math.toRadians(90));

    private static final double SETPOINT_TOLERANCE = 2;

    private boolean parkNavActive = false;
    private boolean parkReached = false;
    private Pose parkSetpoint = new Pose(45.151, 40.329, Math.toRadians(180));
    private Pose gateWaypoint = new Pose(120.73170731707316, 71.34146341463415, Math.toRadians(180));

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
    private static final double AUTO_DOWN_DURATION = 0.15;

    private enum CatapultModes {UP, DOWN, HOLD}
    private CatapultModes pivotMode;

    private enum FootMode {UP, DOWN, BRAKE}
    private FootMode footmode;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        camera = hardwareMap.get(Limelight3A.class, "limelight");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        Pose initialPose = follower.getPose();
        xFilter = new KalmanFilter(0.25, 0.75, initialPose.getX());
        yFilter = new KalmanFilter(0.25, 0.75, initialPose.getY());
        headingFilter = new KalmanFilter(0.25, 0.75, initialPose.getHeading());

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
        camera.start();

        runtime.reset();
        catatime.reset();
        startupCatapultTimer.reset();
        startupCatapultActive = true;
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        if (useLimelightOdometry) {
            updateOdometryFromLimelight();
        }

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
            headingFilter.reset(Math.toRadians(180));
        }

        if (gamepad1.start) {
            Pose currentPose = follower.getPose();
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), 0));
            headingFilter.reset(0);
        }

        boolean driverInput = Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1;

        boolean anyLaunchNavActive = setpointNavActive || targetOneNavActive || targetTwoNavActive || gateNavActive;

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

        if (setpointNavActive && driverInput) {
            follower.startTeleopDrive();
            setpointNavActive = false;
            setpointReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = false;
        }

        if (targetOneNavActive && driverInput) {
            follower.startTeleopDrive();
            targetOneNavActive = false;
            targetOneReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = false;
        }

        if (targetTwoNavActive && driverInput) {
            follower.startTeleopDrive();
            targetTwoNavActive = false;
            targetTwoReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = false;
        }

        if (gateNavActive && driverInput) {
            follower.startTeleopDrive();
            gateNavActive = false;
            gateReached = false;
            launchState = LaunchState.IDLE;
            automatedDrive = false;
        }

        if (parkNavActive && driverInput) {
            follower.startTeleopDrive();
            parkNavActive = false;
            parkReached = false;
            parkFootActive = false;
            automatedDrive = false;
        }

        if (setpointNavActive && !setpointReached) {
            double distanceToTarget = Math.hypot(
                    follower.getPose().getX() - targetSetpoint.getX(),
                    follower.getPose().getY() - targetSetpoint.getY()
            );

            if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
                setpointReached = true;

                if (useLimelightOdometry) {
                    updateOdometryFromLimelight();
                    telemetry.addData("LL Update", "Position corrected at Y setpoint");
                }

                launchState = LaunchState.LAUNCHING_UP;
                setpointLaunchTimer.reset();
            }
        }

        if (targetOneNavActive && !targetOneReached) {
            double distanceToTarget = Math.hypot(
                    follower.getPose().getX() - targetOneSetpoint.getX(),
                    follower.getPose().getY() - targetOneSetpoint.getY()
            );

            if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
                targetOneReached = true;

                if (useLimelightOdometry) {
                    updateOdometryFromLimelight();
                    telemetry.addData("LL Update", "Position corrected at A setpoint");
                }

                launchState = LaunchState.LAUNCHING_UP;
                setpointLaunchTimer.reset();
            }
        }

        if (targetTwoNavActive && !targetTwoReached) {
            double distanceToTarget = Math.hypot(
                    follower.getPose().getX() - targetTwoSetpoint.getX(),
                    follower.getPose().getY() - targetTwoSetpoint.getY()
            );

            if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
                targetTwoReached = true;

                if (useLimelightOdometry) {
                    updateOdometryFromLimelight();
                    telemetry.addData("LL Update", "Position corrected at X setpoint");
                }

                launchState = LaunchState.LAUNCHING_UP;
                setpointLaunchTimer.reset();
            }
        }

        if (gateNavActive && !gateReached) {
            double distanceToTarget = Math.hypot(
                    follower.getPose().getX() - gateSetpoint.getX(),
                    follower.getPose().getY() - gateSetpoint.getY()
            );

            if (distanceToTarget < SETPOINT_TOLERANCE && !follower.isBusy()) {
                gateReached = true;

                if (useLimelightOdometry) {
                    updateOdometryFromLimelight();
                    telemetry.addData("LL Update", "Position corrected at B setpoint");
                }

                launchState = LaunchState.LAUNCHING_UP;
                setpointLaunchTimer.reset();
            }
        }

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

        if (!automatedDrive && !anyLaunchNavActive && !parkNavActive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        }

        if (automatedDrive && !anyLaunchNavActive && !parkNavActive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.left_bumper) {
            slowMode = !slowMode;
        }

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

        boolean canSeeAprilTag = false;
        int visibleTagCount = 0;
        try {
            LLResult result = camera.getLatestResult();
            if (result != null && result.isValid()) {
                visibleTagCount = result.getBotposeTagCount();
                canSeeAprilTag = visibleTagCount > 0;
            }
        } catch (Exception e) {
        }

        telemetry.addData("AprilTag Visible", canSeeAprilTag ? "YES" : "NO");
        telemetry.addData("Tags Detected", visibleTagCount);
        telemetry.addData("Limelight Odom", useLimelightOdometry ? "ENABLED" : "DISABLED");
        telemetry.addData("Position", follower.getPose());
        telemetry.update();
        telemetryM.debug("position", follower.getPose());
    }


    private void updateOdometryFromLimelight() {
        try {
            // Get current odometry prediction from wheels
            Pose currentPose = follower.getPose();

            LLResult result = camera.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    int tagCount = result.getBotposeTagCount();

                    if (tagCount > 0) {
                        double x = botpose.getPosition().x;
                        double y = botpose.getPosition().y;
                        double heading = Math.toRadians(botpose.getOrientation().getYaw());

                        Pose limelightPose = new Pose(x, y, heading, FTCCoordinates.INSTANCE)
                                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                        // Kalman filter fuses odometry prediction with Limelight measurement
                        double fusedX = xFilter.update(limelightPose.getX(), currentPose.getX());
                        double fusedY = yFilter.update(limelightPose.getY(), currentPose.getY());
                        double fusedHeading = headingFilter.update(limelightPose.getHeading(), currentPose.getHeading());

                        follower.setPose(new Pose(fusedX, fusedY, fusedHeading));
                    }
                }
            }
        } catch (Exception e) {
        }
    }
}