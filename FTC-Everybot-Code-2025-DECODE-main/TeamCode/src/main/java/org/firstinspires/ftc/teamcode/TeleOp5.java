package org.firstinspires.ftc.teamcode;

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
@TeleOp(name = "TeleOp5", group = "Teleop")
public class TeleOp5 extends OpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();
    private ElapsedTime autoDownTimer = new ElapsedTime();

    // Pedro Pathing drive system
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    // Heading lock variables
    private boolean headingLockEnabled = false;
    private boolean lastXState = false;
    private final double LOCKED_HEADING = Math.toRadians(50); // 50 degrees in radians

    // Auto-drive to position variables
    private boolean autoDriveActive = false;
    private boolean lastYState = false;
    private final Pose TARGET_POSE = new Pose(0, 0, Math.toRadians(0)); // Target: x=0, y=0, heading=0°
    private PathChain autoDrivePath;

    // Button debouncing
    private boolean lastDpadUpState = false;

    // Subsystem motors
    private DcMotor intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private DcMotor foot = null;


    // Intake power constants
    private double INTAKE_IN_POWER = -1;
    private double INTAKE_OUT_POWER = -0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    // Foot power constants
    private double FOOT_DOWN_POWER = 1.0;

    private double FOOT_UP_POWER = -1.0;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;

    // Catapult power constants
    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
    private double CATAPULT_HOLD_POWER = 0.2;

    // Auto catapult down variables
    private boolean autoDownActive = false;
    private boolean wasUpButtonPressed = false;
    private static final double AUTO_DOWN_DURATION = 1.0; // 1 second

    // Enums for subsystem states
    private enum CatapultModes {UP, DOWN, HOLD}
    private CatapultModes pivotMode;

    private enum FootMode {DOWN, BRAKE, UP}
    private FootMode footmode;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize automated path (example path)
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        // Initialize subsystem motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");
        foot = hardwareMap.get(DcMotor.class, "foot");

        // Set motor directions
        intake.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        foot.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
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
        // Update Pedro Pathing
        follower.update();
        telemetryM.update();

        // HEADING LOCK TOGGLE (X button)
        boolean currentXState = gamepad1.x;
        if (currentXState && !lastXState) {
            // Toggle heading lock on button press
            headingLockEnabled = !headingLockEnabled;
            autoDriveActive = false; // Disable auto-drive if heading lock is enabled
        }
        lastXState = currentXState;

        // AUTO-DRIVE TO POSITION (Y button)
        boolean currentYState = gamepad1.y;
        if (currentYState && !lastYState) {
            // Toggle auto-drive on button press
            autoDriveActive = !autoDriveActive;
            headingLockEnabled = false; // Disable heading lock if auto-drive is enabled

            if (autoDriveActive) {
                // Build path from current position to target
                autoDrivePath = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), TARGET_POSE))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), TARGET_POSE.getHeading())
                        .build();
                follower.followPath(autoDrivePath, true);
            }
        }
        lastYState = currentYState;

        // Check if auto-drive path is complete
        if (autoDriveActive && !follower.isBusy()) {
            autoDriveActive = false; // Path completed, return to manual control
        }

        // DRIVE CODE - Pedro Pathing
        if (autoDriveActive) {
            // Auto-drive mode: Let Pedro handle everything
            // No manual input, robot drives to target automatically
        } else if (!automatedDrive) {
            if (headingLockEnabled) {
                // Heading lock mode: control translation but lock heading to 50 degrees
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        0, // No rotation input from right stick
                        false // Robot Centric
                );

                // Use Pedro's heading correction to maintain 50 degrees
                follower.setMaxPower(0.8); // Slightly reduce power for smoother heading correction
                double currentHeading = follower.getPose().getHeading();
                double headingError = LOCKED_HEADING - currentHeading;

                // Normalize heading error to [-PI, PI]
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;

                // Apply corrective rotation using a proportional controller
                double kP = 1.5; // Proportional gain for heading correction
                double correctionPower = headingError * kP;

                // Clamp correction power
                correctionPower = Math.max(-0.6, Math.min(0.6, correctionPower));

                // Override with correction
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        correctionPower,
                        false
                );
            } else {
                // Normal drive mode
                follower.setMaxPower(1.0);
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // Robot Centric
                );
            }
        }

        // READ BUTTONS FOR SUBSYSTEMS
        boolean intakeInButton = gamepad1.left_trigger > 0.2;
        boolean intakeOutButton = gamepad1.left_bumper;
        boolean footDownButton = gamepad1.dpad_down;
        boolean footUpButton = gamepad1.dpad_up;


        boolean catapultUpButton = gamepad1.right_trigger > 0.2;

        // AUTO CATAPULT DOWN LOGIC
        // Detect when up button is released
        if (wasUpButtonPressed && !catapultUpButton && !autoDownActive) {
            autoDownActive = true;
            autoDownTimer.reset();
        }

        // Update the previous state
        wasUpButtonPressed = catapultUpButton;

        // Check if auto down timer has expired
        if (autoDownActive && autoDownTimer.seconds() >= AUTO_DOWN_DURATION) {
            autoDownActive = false;
        }

        // INTAKE CONTROL
        if (intakeInButton) {
            intakePower = INTAKE_IN_POWER;
        } else if (intakeOutButton) {
            intakePower = 1.0;
        } else {
            intakePower = INTAKE_OFF_POWER;
        }

        // FOOT CONTROL
        if (footDownButton) {
            footmode = FootMode.DOWN;
            footPower = FOOT_DOWN_POWER;
        } else if (footUpButton){
            footmode = FootMode.UP;
            footPower = FOOT_UP_POWER;

        } else {
            footmode = FootMode.BRAKE;
            footPower = FOOT_OFF_POWER;
        }

        // CATAPULT CONTROL
        if (catapultUpButton) {
            pivotMode = CatapultModes.UP;
            catapult1.setPower(CATAPULT_UP_POWER);
            catapult2.setPower(CATAPULT_UP_POWER);
        } else if (autoDownActive) {
            // Auto down after releasing up button
            pivotMode = CatapultModes.DOWN;
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
        } else {
            pivotMode = CatapultModes.HOLD;
            catapult1.setPower(CATAPULT_HOLD_POWER);
            catapult2.setPower(CATAPULT_HOLD_POWER);
        }

        // SET SUBSYSTEM MOTOR POWERS
        intake.setPower(intakePower);
        foot.setPower(footPower);

        // TELEMETRY
        String catapult_mode_str;
        if (pivotMode == CatapultModes.UP) {
            catapult_mode_str = "UP";
        } else if (pivotMode == CatapultModes.DOWN) {
            catapult_mode_str = "DOWN";
        } else {
            catapult_mode_str = "HOLD";
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Auto-Drive", autoDriveActive ? "ACTIVE → (0,0,0°)" : "DISABLED");
        telemetry.addData("Heading Lock", headingLockEnabled ? "ENABLED (50°)" : "DISABLED");
        telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
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
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("headingLock", headingLockEnabled);
        telemetryM.debug("autoDrive", autoDriveActive);
    }
}