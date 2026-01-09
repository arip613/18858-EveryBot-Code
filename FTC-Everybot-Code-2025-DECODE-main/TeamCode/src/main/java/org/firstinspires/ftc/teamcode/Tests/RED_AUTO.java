package org.firstinspires.ftc.teamcode.Tests;

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

import org.firstinspires.ftc.teamcode.Teleop.RED_TELEOP;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "12C", group = "Examples")
public class RED_AUTO extends OpMode {

    private Follower follower;
    private Timer waitTimer;

    private Timer pathTimer, opmodeTimer;
    private TelemetryManager telemetryM;

    private DcMotor intake = null;
    private double INTAKE_IN_POWER = -0.75;
    private double INTAKE_OFF_POWER = 0.0;

    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;

    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
    private double CATAPULT_HOLD_POWER = 0.15;

    private enum CatapultModes {UP, DOWN, HOLD}
    private CatapultModes catapultMode = CatapultModes.HOLD;

    private Timer catapultUpTimer;
    private Timer catapultDownTimer;
    private Timer gateWaitTimer;

    private int pathState;
    private final Pose startPose = new Pose(111.232, 135.752, Math.toRadians(90));
    private final Pose scorePose = new Pose(116.45412130637635, 126.97978227060653, Math.toRadians(37));

    private final Pose pickup1Pose = new Pose(95, 88, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(129, 88, Math.toRadians(0));

    private final Pose pickup2Pose = new Pose(95, 63, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(135, 63, Math.toRadians(0));
    private final Pose pickup2AvoidPose = new Pose(95, 88, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(95, 41, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(135, 41, Math.toRadians(0));

    private final Pose gateStartPose = new Pose(118.03732503888025, 63, Math.toRadians(0));
    private final Pose gateEndPose = new Pose(128.99533437013997, 69.87247278382581, Math.toRadians(0));

    private PathChain startToScore;
    private PathChain scoreToPickup2Avoid, pickup2AvoidToPickup2, pickup2ToPickup2End;
    private PathChain pickup2EndToGateStart, gateStartToGateEnd, gateEndToPickup2;
    private PathChain pickup2ToPickup2Avoid, pickup2AvoidToScore;
    private PathChain scoreToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End, pickup3EndToPickup3, pickup3ToScore;
    private PathChain scoreToPickup2;

    public void buildPaths() {
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        scoreToPickup2Avoid = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2AvoidPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2AvoidPose.getHeading())
                .build();

        pickup2AvoidToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2AvoidPose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2AvoidPose.getHeading(), pickup2Pose.getHeading())
                .build();

        pickup2ToPickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())
                .setVelocityConstraint(0.5)
                .build();

        pickup2EndToGateStart = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, gateStartPose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), gateStartPose.getHeading())
                .build();

        gateStartToGateEnd = follower.pathBuilder()
                .addPath(new BezierLine(gateStartPose, gateEndPose))
                .setLinearHeadingInterpolation(gateStartPose.getHeading(), gateEndPose.getHeading())
                .build();

        gateEndToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(gateEndPose, pickup2Pose))
                .setLinearHeadingInterpolation(gateEndPose.getHeading(), pickup2Pose.getHeading())
                .build();

        pickup2ToPickup2Avoid = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2AvoidPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2AvoidPose.getHeading())
                .build();

        pickup2AvoidToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickup2AvoidPose, scorePose))
                .setLinearHeadingInterpolation(pickup2AvoidPose.getHeading(), scorePose.getHeading())
                .build();

        scoreToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        pickup1ToPickup1End = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .setVelocityConstraint(0.5)
                .build();

        pickup1EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

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

        scoreToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
    }


    private void launch() {
        catapultMode = CatapultModes.UP;
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
        catapultUpTimer.resetTimer();
    }

    private void updateCatapult() {
        if (catapultMode == CatapultModes.UP && catapultUpTimer.getElapsedTimeSeconds() > 0.25) {
            catapultMode = CatapultModes.DOWN;
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
            catapultDownTimer.resetTimer();
        } else if (catapultMode == CatapultModes.DOWN && catapultDownTimer.getElapsedTimeSeconds() > 0.35) {
            catapultMode = CatapultModes.HOLD;
            catapult1.setPower(CATAPULT_HOLD_POWER);
            catapult2.setPower(CATAPULT_HOLD_POWER);
        }
    }

    private boolean isCatapultReady() {
        return catapultMode == CatapultModes.HOLD;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start -> Score Pose
                follower.followPath(startToScore, true);
                intake.setPower(INTAKE_IN_POWER);
                waitTimer.resetTimer();
                setPathState(1);
                break;

            case 1:
                // Launch
                if(!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 1) {
                    launch();
                    setPathState(2);
                }
                break;

            case 2:
                // Launch Again
                if(isCatapultReady()) {
                    launch();
                    setPathState(3);
                }
                break;

            case 3:
                // Score Pose to Pickup2AvoidPose
                if(isCatapultReady()) {
                    follower.followPath(scoreToPickup2Avoid, true);
                    setPathState(4);
                }
                break;

            case 4:
                // Pickup2AvoidPose to Pickup2Pose
                if(!follower.isBusy()) {
                    follower.followPath(pickup2AvoidToPickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Pickup2 to Pickup2EndPose
                if(!follower.isBusy()) {
                    follower.followPath(pickup2ToPickup2End, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(pickup2EndToGateStart, true);
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(gateStartToGateEnd, true);
                    waitTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                // GateEndPose to Pickup2
                if(waitTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(gateEndToPickup2, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Pickup2 to Pickup2AvoidPose
                if(!follower.isBusy()) {
                    follower.followPath(pickup2ToPickup2Avoid, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Pickup2AvoidPose to Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup2AvoidToScore, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Launch
                if(!follower.isBusy()) {
                    launch();
                    setPathState(12);
                }
                break;

            case 12:
                // Score to Pickup1
                if(isCatapultReady()) {
                    follower.followPath(scoreToPickup1, true);
                    setPathState(13);
                }
                break;

            case 13:
                // Pickup1 to Pickup1End
                if(!follower.isBusy()) {
                    follower.followPath(pickup1ToPickup1End, true);
                    setPathState(14);
                }
                break;

            case 14:
                // Pickup1End to Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup1EndToScore, true);
                    setPathState(15);
                    waitTimer.resetTimer();
                }
                break;
            case 15:
                if(!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 0.5) {
                    launch();
                    setPathState(16);
                }
                break;


            case 16:
                if(isCatapultReady()) {
                    follower.followPath(scoreToPickup3, true);
                    setPathState(17);
                }
                break;

            case 17:
                // Pickup3 to Pickup3End
                if(!follower.isBusy()) {
                    follower.followPath(pickup3ToPickup3End, true);
                    setPathState(18);
                }
                break;

            case 18:
                // Pickup3End to Pickup3
                if(!follower.isBusy()) {
                    follower.followPath(pickup3EndToPickup3, true);
                    setPathState(19);
                }
                break;

            case 19:
                // Pickup3 to Score
                if(!follower.isBusy()) {
                    follower.followPath(pickup3ToScore, true);
                    setPathState(20);
                }
                break;

            case 20:
                // Launch
                if(!follower.isBusy()) {
                    launch();
                    setPathState(21);
                }
                break;

            case 21:
                // Score to Pickup2Pose
                if(isCatapultReady()) {
                    follower.followPath(scoreToPickup2, true);
                    setPathState(22);
                }
                break;

            case 22:
                // End
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
        updateCatapult();
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
        waitTimer = new Timer();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        catapultUpTimer = new Timer();
        catapultDownTimer = new Timer();
        gateWaitTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        buildPaths();
        follower.setStartingPose(startPose);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        thug.startingPose = follower.getPose();
    }
}