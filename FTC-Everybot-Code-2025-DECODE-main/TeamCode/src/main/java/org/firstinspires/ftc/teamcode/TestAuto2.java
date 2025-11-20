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

@Autonomous(name = "Auto2", group = "Examples")
public class TestAuto2 extends OpMode {

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
    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
    private double CATAPULT_HOLD_POWER = 0.2;

    // Catapult timing constants
    private static final double CATAPULT_UP_DURATION = 0.5; // Time to hold catapult up
    private static final double CATAPULT_DOWN_DURATION = 1.0; // Time for auto down
    private static final double WAIT_BETWEEN_SHOTS = 0.5; // Wait time between two shots

    private int pathState;
    private final Pose startPose = new Pose(120, 132, Math.toRadians(50));
    private final Pose scorePose = new Pose(120, 132, Math.toRadians(50));

    private final Pose pickup0Pose = new Pose(112, 112, Math.toRadians(50));

    // Pickup 1 poses
    private final Pose pickup1Pose = new Pose(108, 84, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(135, 84, Math.toRadians(0));

    // Pickup 2 poses
    private final Pose pickup2Pose = new Pose(108, 60, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(144, 60, Math.toRadians(0));

    // Pickup 3 poses
    private final Pose pickup3Pose = new Pose(108, 36, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(144, 36, Math.toRadians(0));

    // Path declarations
    private PathChain startToPickup0;
    private PathChain pickup0ToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup2, pickup2ToPickup2End, pickup2EndToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End, pickup3EndToScore;

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

        pickup2EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePose.getHeading())
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

        pickup3EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePose.getHeading())
                .build();
    }

    // Method to fire catapult (simulates right trigger press)
    private void fireCatapult() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
    }

    private void catapultDown() {
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
    }

    private void catapultHold() {
        catapult1.setPower(CATAPULT_HOLD_POWER);
        catapult2.setPower(CATAPULT_HOLD_POWER);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start -> Pickup 0
                follower.followPath(startToPickup0, true);
                intake.setPower(INTAKE_OFF_POWER);
                setPathState(1);
                break;

            case 1:
                // Wait at Pickup 0, then fire catapult (first shot)
                if(!follower.isBusy()) {
                    fireCatapult();
                    setPathState(2);
                }
                break;

            case 2:
                // Hold catapult up for 0.5 seconds
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_UP_DURATION) {
                    catapultDown();
                    setPathState(3);
                }
                break;

            case 3:
                // Auto down for 1 second
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_DOWN_DURATION) {
                    catapultHold();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait 0.5 seconds between shots
                if(pathTimer.getElapsedTimeSeconds() >= WAIT_BETWEEN_SHOTS) {
                    fireCatapult();
                    setPathState(5);
                }
                break;

            case 5:
                // Hold catapult up for 0.5 seconds (second shot)
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_UP_DURATION) {
                    catapultDown();
                    setPathState(6);
                }
                break;

            case 6:
                // Auto down for 1 second
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_DOWN_DURATION) {
                    catapultHold();
                    // Turn on intake for rest of code
                    intake.setPower(INTAKE_IN_POWER);
                    follower.followPath(pickup0ToPickup1, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Pickup 0 -> Pickup 1, then to Pickup 1 End
                if(!follower.isBusy()) {
                    follower.followPath(pickup1ToPickup1End, true);
                    setPathState(8);
                }
                break;

            case 8:
                // Pickup 1 End -> Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup1EndToScore, true);
                    setPathState(9);
                }
                break;

            case 9:
                // At score position, fire catapult
                if(!follower.isBusy()) {
                    fireCatapult();
                    setPathState(10);
                }
                break;

            case 10:
                // Hold catapult up
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_UP_DURATION) {
                    catapultDown();
                    setPathState(11);
                }
                break;

            case 11:
                // Auto down
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_DOWN_DURATION) {
                    catapultHold();
                    follower.followPath(scoreToPickup2, true);
                    setPathState(12);
                }
                break;

            case 12:
                // Score -> Pickup 2, then to Pickup 2 End
                if(!follower.isBusy()) {
                    follower.followPath(pickup2ToPickup2End, true);
                    setPathState(13);
                }
                break;

            case 13:
                // Pickup 2 End -> Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup2EndToScore, true);
                    setPathState(14);
                }
                break;

            case 14:
                // At score position, fire catapult
                if(!follower.isBusy()) {
                    fireCatapult();
                    setPathState(15);
                }
                break;

            case 15:
                // Hold catapult up
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_UP_DURATION) {
                    catapultDown();
                    setPathState(16);
                }
                break;

            case 16:
                // Auto down
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_DOWN_DURATION) {
                    catapultHold();
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
                // Pickup 3 End -> Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup3EndToScore, true);
                    setPathState(19);
                }
                break;

            case 19:
                // At score position, fire catapult (final shot)
                if(!follower.isBusy()) {
                    fireCatapult();
                    setPathState(20);
                }
                break;

            case 20:
                // Hold catapult up
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_UP_DURATION) {
                    catapultDown();
                    setPathState(21);
                }
                break;

            case 21:
                // Auto down
                if(pathTimer.getElapsedTimeSeconds() >= CATAPULT_DOWN_DURATION) {
                    catapultHold();
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
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