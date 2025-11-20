

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
@TeleOp(name = "Test Teleop", group = "Teleop")
public class TeleOp1 extends OpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();

    // Pedro Pathing drive system
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Subsystem motors
    private DcMotor intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private DcMotor foot = null;

    // Intake power constants
    private double INTAKE_IN_POWER = -0.6;
    private double INTAKE_OUT_POWER = -0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    // Foot power constants
    private double FOOT_UP_POWER = 1.0;
    private double FOOT_DOWN_POWER = -0.85;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;

    // Catapult power constants
    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
    private double CATAPULT_HOLD_POWER = 0.2;

    // Enums for subsystem states
    private enum CatapultModes {UP, DOWN, HOLD}
    private CatapultModes pivotMode;

    private enum FootMode {UP, DOWN, BRAKE}
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

        // DRIVE CODE - Pedro Pathing
        if (!automatedDrive) {
            if (!slowMode) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // Robot Centric
                );
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        false // Robot Centric
                );
            }
        }

        // Automated PathFollowing (optional - triggered by dpad_up)
        if (gamepad1.dpad_up) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        // Stop automated following
        if (automatedDrive && (gamepad1.dpad_down || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Slow Mode toggle
        if (gamepad1.right_bumper) {
            slowMode = !slowMode;
        }

        // READ BUTTONS FOR SUBSYSTEMS
        boolean intakeInButton = gamepad1.left_trigger > 0.2;
   /*     boolean intakeOutButton = gamepad1.left_bumper;

        // Resolve conflicting intake buttons
        if (intakeInButton && intakeOutButton) {
            intakeInButton = false;
        }

    */

        boolean footOutButton = gamepad1.a;
        boolean footUpButton = gamepad1.b;

        // Resolve conflicting foot buttons
        if (footOutButton && footUpButton) {
            footOutButton = false;
        }

        boolean catapultUpButton = gamepad1.right_bumper;
        boolean catapultDownButton = gamepad1.right_trigger > 0.2;

        // Resolve conflicting catapult buttons
        if (catapultUpButton && catapultDownButton) {
            catapultUpButton = false;
        }

        // INTAKE CONTROL
        if (intakeInButton) {
            intakePower = INTAKE_IN_POWER;
        } else {
            intakePower = INTAKE_OFF_POWER;
        }

        // FOOT CONTROL
        if (footOutButton) {
            footmode = FootMode.DOWN;
            footPower = FOOT_DOWN_POWER;
        } else {
            footmode = FootMode.BRAKE;
            footPower = FOOT_OFF_POWER;
        }

        // CATAPULT CONTROL
        if (catapultUpButton) {
            pivotMode = CatapultModes.UP;
            catapult1.setPower(CATAPULT_UP_POWER);
            catapult2.setPower(CATAPULT_UP_POWER);
        } else if (catapultDownButton) {
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
        telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Automated Drive", automatedDrive);
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
    }
}