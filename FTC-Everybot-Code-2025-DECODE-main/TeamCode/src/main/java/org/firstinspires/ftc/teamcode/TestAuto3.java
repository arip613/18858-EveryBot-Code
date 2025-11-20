package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto3", group = "Examples")
public class TestAuto3 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private TelemetryManager telemetryM;

    // Intake motor
    private DcMotor intake = null;
    private double INTAKE_IN_POWER = -0.75;
    private double INTAKE_OFF_POWER = 0.0;

    // Catapult motors
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;

    // Catapult stage constants
    private static final int STAGE_0 = 0; // Launching stage
    private static final int STAGE_1 = 1; // Loading/Holding stage

    // Power constants for catapult
    private double CATAPULT_STAGE_0_POWER = 1.0;
    private double CATAPULT_STAGE_1_POWER = 0.3;

    // Encoder constants - VERIFY THESE VALUES FOR YOUR MOTORS!
    private static final int TICKS_PER_REVOLUTION =  100; // Adjust based on your motor
    private static final int STAGE_0_ROTATIONS = 0; // Stage 0 position
    private static final int STAGE_1_ROTATIONS = -1; // Stage 1 is -1 rotation from Stage 0 (reduced from -3)

    // Current catapult stage
    private int catapultStage = STAGE_0;

    private int pathState;
    private final Pose startPose = new Pose(120, 132, Math.toRadians(50));
    private final Pose scorePose = new Pose(120, 120, Math.toRadians(50));

    private final Pose pickup0Pose = new Pose(120, 120, Math.toRadians(50));

    // Pickup 1 poses
    private final Pose pickup1Pose = new Pose(108, 86, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(135, 86, Math.toRadians(0));

    // Pickup 2 poses
    private final Pose pickup2Pose = new Pose(108, 60, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(144, 60, Math.toRadians(0));
    private final Pose pickup2EndAvoidPose = new Pose(112, 84, Math.toRadians(0)); // Avoid obstacle

    // Pickup 3 poses
    private final Pose pickup3Pose = new Pose(108, 36, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(148, 36, Math.toRadians(0));

    // Path declarations
    private PathChain startToPickup0;
    private PathChain pickup0ToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup2, pickup2ToPickup2End, pickup2EndToPickup2EndAvoid, pickup2EndAvoidToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End, pickup3EndToPickup3, pickup3ToScore;

    public void buildPaths() {
        // START TO PICKUP 0
        startToPickup0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup0Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup0Pose.getHeading())
                .build();

        // PICKUP 0 TO PICKUP 1
        pickup0ToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup0Pose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup0Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        // PICKUP 1 SEQUENCE
        pickup1ToPickup1End = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .setVelocityConstraint(1)
                .build();

        pickup1EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

        // PICKUP 2 SEQUENCE
        scoreToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        pickup2ToPickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())
                .setVelocityConstraint(1)
                .build();

        pickup2EndToPickup2EndAvoid = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, pickup2EndAvoidPose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), pickup2EndAvoidPose.getHeading())
                .build();

        pickup2EndAvoidToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndAvoidPose, scorePose))
                .setLinearHeadingInterpolation(pickup2EndAvoidPose.getHeading(), scorePose.getHeading())
                .build();

        // PICKUP 3 SEQUENCE
        scoreToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        pickup3ToPickup3End = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3EndPose.getHeading())
                .setVelocityConstraint(1)
                .build();

        pickup3EndToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, pickup3Pose))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), pickup3Pose.getHeading())
                .build();

        pickup3ToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    // Go to Stage 0 (Launch position)
    private void goToStage0() {
        catapultStage = STAGE_0;

        int targetPosition = STAGE_0_ROTATIONS * TICKS_PER_REVOLUTION;
        catapult1.setTargetPosition(targetPosition);
        catapult2.setTargetPosition(targetPosition);

        catapult1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        catapult1.setPower(CATAPULT_STAGE_0_POWER);
        catapult2.setPower(CATAPULT_STAGE_0_POWER);
    }

    // Go to Stage 1 (Load/Hold position)
    private void goToStage1() {
        catapultStage = STAGE_1;

        int targetPosition = STAGE_1_ROTATIONS * TICKS_PER_REVOLUTION;
        catapult1.setTargetPosition(targetPosition);
        catapult2.setTargetPosition(targetPosition);

        catapult1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        catapult1.setPower(CATAPULT_STAGE_1_POWER);
        catapult2.setPower(CATAPULT_STAGE_1_POWER);
    }

    // Check if catapult has reached target stage
    private boolean isCatapultAtTarget() {
        return !catapult1.isBusy() && !catapult2.isBusy();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start -> Pickup 0
                follower.followPath(startToPickup0, true);
                intake.setPower(INTAKE_IN_POWER);
                setPathState(1);
                break;

            case 1:
                // Wait at Pickup 0, then load (go to Stage 1)
                if(!follower.isBusy()) {
                    goToStage1();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for load to complete, then launch (go to Stage 0)
                if(isCatapultAtTarget()) {
                    goToStage0();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for launch to complete, then load again
                if(isCatapultAtTarget()) {
                    goToStage1();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for load to complete, then launch (second shot)
                if(isCatapultAtTarget()) {
                    goToStage0();
                    setPathState(5);
                }
                break;

            case 5:
                // Wait for launch to complete, turn on intake, move to pickup 1
                if(isCatapultAtTarget()) {
                    intake.setPower(INTAKE_IN_POWER);
                    follower.followPath(pickup0ToPickup1, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Pickup 0 -> Pickup 1, then to Pickup 1 End
                if(!follower.isBusy()) {
                    follower.followPath(pickup1ToPickup1End, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Pickup 1 End -> Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup1EndToScore, true);
                    setPathState(8);
                }
                break;

            case 8:
                // At score position, load
                if(!follower.isBusy()) {
                    goToStage1();
                    setPathState(9);
                }
                break;

            case 9:
                // Wait for load to complete, then launch
                if(isCatapultAtTarget()) {
                    goToStage0();
                    setPathState(10);
                }
                break;

            case 10:
                // Wait for launch to complete, move to pickup 2
                if(isCatapultAtTarget()) {
                    follower.followPath(scoreToPickup2, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Score -> Pickup 2, then to Pickup 2 End
                if(!follower.isBusy()) {
                    follower.followPath(pickup2ToPickup2End, true);
                    setPathState(12);
                }
                break;

            case 12:
                // Pickup 2 End -> Pickup 2 End Avoid
                if(!follower.isBusy()) {
                    follower.followPath(pickup2EndToPickup2EndAvoid, true);
                    setPathState(13);
                }
                break;

            case 13:
                // Pickup 2 End Avoid -> Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup2EndAvoidToScore, true);
                    setPathState(14);
                }
                break;

            case 14:
                // At score position, load
                if(!follower.isBusy()) {
                    goToStage1();
                    setPathState(15);
                }
                break;

            case 15:
                // Wait for load to complete, then launch
                if(isCatapultAtTarget()) {
                    goToStage0();
                    setPathState(16);
                }
                break;

            case 16:
                // Wait for launch to complete, move to pickup 3
                if(isCatapultAtTarget()) {
                    follower.followPath(scoreToPickup3, true);
                    setPathState(17);
                }
                break;

            case 17:
                // Score -> Pickup 3, then to Pickup 3 End
                if(!follower.isBusy()) {
                    follower.followPath(pickup3ToPickup3End, true);
                    setPathState(18);
                }
                break;

            case 18:
                // Pickup 3 End -> Pickup 3
                if(!follower.isBusy()) {
                    follower.followPath(pickup3EndToPickup3, true);
                    setPathState(19);
                }
                break;

            case 19:
                // Pickup 3 -> Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup3ToScore, true);
                    setPathState(20);
                }
                break;

            case 20:
                // At score position, load
                if(!follower.isBusy()) {
                    goToStage1();
                    setPathState(21);
                }
                break;

            case 21:
                // Wait for load to complete, then launch (final shot)
                if(isCatapultAtTarget()) {
                    goToStage0();
                    setPathState(22);
                }
                break;

            case 22:
                // Wait for final launch to complete
                if(isCatapultAtTarget()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetryM.update();

        // Telemetry
        telemetry.addData("path state", pathState);
        telemetry.addData("catapult stage", catapultStage);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Catapult1 Pos", catapult1.getCurrentPosition());
        telemetry.addData("Catapult2 Pos", catapult2.getCurrentPosition());
        telemetry.addData("Catapult1 Target", catapult1.getTargetPosition());
        telemetry.addData("Catapult2 Target", catapult2.getTargetPosition());
        telemetry.addData("Catapult1 Power", catapult1.getPower());
        telemetry.addData("Catapult2 Power", catapult2.getPower());
        telemetry.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("path state", pathState);
        telemetryM.debug("catapult stage", catapultStage);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        buildPaths();
        follower.setStartingPose(startPose);

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize catapult motors
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and configure encoders
        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Start at Stage 0 (launch position)
        goToStage0();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intake.setPower(INTAKE_IN_POWER);
        setPathState(0);
    }

    @Override
    public void stop() {
        intake.setPower(0);
        catapult1.setPower(0);
        catapult2.setPower(0);
        TeleOp1.startingPose = follower.getPose();
    }
}