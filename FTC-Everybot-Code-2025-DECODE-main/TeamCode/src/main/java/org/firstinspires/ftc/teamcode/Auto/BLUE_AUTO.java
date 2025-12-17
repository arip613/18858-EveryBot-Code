package org.firstinspires.ftc.teamcode.Auto;

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

import org.firstinspires.ftc.teamcode.Teleop.BLUE_TELEOP;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto", group = "Examples")
public class BLUE_AUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private TelemetryManager telemetryM;

    // Intake motor
    private DcMotor intake = null;
    private double INTAKE_IN_POWER = -0.75;
    private double INTAKE_OFF_POWER = 0.0;

    // Catapult motors
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;

    // Power constants for catapult
    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
    private double CATAPULT_HOLD_POWER = 0.15;

    // Catapult mode enum
    private enum CatapultModes {UP, DOWN, HOLD}
    private CatapultModes catapultMode = CatapultModes.HOLD;

    // Catapult timers
    private Timer catapultUpTimer;
    private Timer catapultDownTimer;

    private int pathState;
    private final Pose startPose = new Pose(111.232, 135.752, Math.toRadians(90)).mirror();
    private final Pose scorePose = new Pose(117.21951219512195, 118.7560975609756, Math.toRadians(37)).mirror();

    private final Pose pickup0Pose= new Pose(113.92682926829268, 121.17073170731707, Math.toRadians(37)).mirror();

    private final Pose pickup1Pose = new Pose(95, 88, Math.toRadians(0)).mirror();
    private final Pose pickup1EndPose = new Pose(128, 88, Math.toRadians(0)).mirror();

    private final Pose pickup2Pose = new Pose(95, 65, Math.toRadians(0)).mirror();
    private final Pose pickup2EndPose = new Pose(134, 65, Math.toRadians(0)).mirror();
    private final Pose pickup2EndAvoidPose = new Pose(112, 60, Math.toRadians(0)).mirror();

    private final Pose pickup3Pose = new Pose(95, 41, Math.toRadians(0)).mirror();
    private final Pose pickup3EndPose = new Pose(134, 41, Math.toRadians(0)).mirror();

    private PathChain startToPickup0;
    private PathChain pickup0ToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup2, pickup2ToPickup2End, pickup2EndToPickup2EndAvoid, pickup2EndAvoidToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End, pickup3EndToPickup3, pickup3ToScore, pickup3EndToPickup2;

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
                .setVelocityConstraint(0.5)
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
                .setVelocityConstraint(0.5)
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
                .setVelocityConstraint(0.5)
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

    // Launch the catapult - starts the UP->DOWN->HOLD sequence
    private void launch() {
        catapultMode = CatapultModes.UP;
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
        catapultUpTimer.resetTimer();
    }

    private void updateCatapult() {
        if (catapultMode == CatapultModes.UP && catapultUpTimer.getElapsedTimeSeconds() > 0.15) {
            catapultMode = CatapultModes.DOWN;
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
            catapultDownTimer.resetTimer();
        } else if (catapultMode == CatapultModes.DOWN && catapultDownTimer.getElapsedTimeSeconds() > 0.2) {
            catapultMode = CatapultModes.HOLD;
            catapult1.setPower(CATAPULT_HOLD_POWER);
            catapult2.setPower(CATAPULT_HOLD_POWER);
        }
    }

    // Check if catapult has completed its launch sequence and is in HOLD
    private boolean isCatapultReady() {
        return catapultMode == CatapultModes.HOLD;
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
                // Wait at Pickup 0, then launch first shot
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.35) {
                    launch();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for first launch to complete, then launch second shot
                if(isCatapultReady()) {
                    launch();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for second launch to complete, turn on intake, move to pickup 1
                if(isCatapultReady()) {
                    intake.setPower(INTAKE_IN_POWER);
                    follower.followPath(pickup0ToPickup1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(pickup1ToPickup1End, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(pickup1EndToScore, true);
                    setPathState(6);
                }
                break;

            case 6:
                // At score position, launch
                if(!follower.isBusy()) {
                    launch();
                    setPathState(7);
                }
                break;

            case 7:
                if(isCatapultReady()) {
                    follower.followPath(scoreToPickup2, true);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(pickup2ToPickup2End, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Pickup 2 End -> Pickup 2 End Avoid
                if(!follower.isBusy()) {
                    follower.followPath(pickup2EndToPickup2EndAvoid, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Pickup 2 End Avoid -> Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup2EndAvoidToScore, true);
                    setPathState(11);
                }
                break;

            case 11:
                // At score position, launch
                if(!follower.isBusy()) {
                    launch();
                    setPathState(12);
                }
                break;

            case 12:
                // Wait for launch to complete, move to pickup 3
                if(isCatapultReady()) {
                    follower.followPath(scoreToPickup3, true);
                    setPathState(13);
                }
                break;

            case 13:
                // Score -> Pickup 3, then to Pickup 3 End
                if(!follower.isBusy()) {
                    follower.followPath(pickup3ToPickup3End, true);
                    setPathState(14);
                }
                break;

            case 14:
                // Pickup 3 End -> Pickup 3
                if(!follower.isBusy()) {
                    follower.followPath(pickup3EndToPickup3, true);
                    setPathState(15);
                }
                break;

            case 15:
                // Pickup 3 -> Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup3ToScore, true);
                    setPathState(16); //jerk off really hard
                }
                break;

            case 16:
                // At score position, launch final shot
                if(!follower.isBusy()) {
                    launch();
                    setPathState(17);
                }
                break;

            case 17:
                if(isCatapultReady()) {
                    follower.followPath(scoreToPickup2, true);
                    setPathState(18);
                }
                break;

            case 18:
                if(!follower.isBusy()) {
                    intake.setPower(INTAKE_OFF_POWER);
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
        updateCatapult();  // Update catapult state machine
        autonomousPathUpdate();
        telemetryM.update();

        // Telemetry
        telemetry.addData("path state", pathState);
        telemetry.addData("catapult mode", catapultMode);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Catapult1 Power", catapult1.getPower());
        telemetry.addData("Catapult2 Power", catapult2.getPower());
        telemetry.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("path state", pathState);
        telemetryM.debug("catapult mode", catapultMode);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        catapultUpTimer = new Timer();
        catapultDownTimer = new Timer();
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

        // Start in HOLD mode
        catapultMode = CatapultModes.HOLD;
        catapult1.setPower(CATAPULT_HOLD_POWER);
        catapult2.setPower(CATAPULT_HOLD_POWER);

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
        BLUE_TELEOP.startingPose = follower.getPose();
    }
}
