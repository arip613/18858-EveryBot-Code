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

import org.firstinspires.ftc.teamcode.Teleop.RED_TELEOP;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto", group = "Examples")
public class RED_AUTO extends OpMode {

    private Follower follower;
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

    private int pathState;
    private final Pose startPose = new Pose(105.805, 134.780, Math.toRadians(90));
    private final Pose scorePose = new Pose(118.537, 119.634, Math.toRadians(37));

    private final Pose pickup0Pose= new Pose(118.537, 119.634, Math.toRadians(37));

    private final Pose pickup1Pose = new Pose(95, 84, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(128, 84, Math.toRadians(0));

    private final Pose pickup2Pose = new Pose(95, 60, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(134, 60, Math.toRadians(0));
    private final Pose pickup2EndAvoidPose = new Pose(112, 60, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(95, 37, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(134, 37, Math.toRadians(0));

    private PathChain startToPickup0;
    private PathChain pickup0ToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup2, pickup2ToPickup2End, pickup2EndToPickup2EndAvoid, pickup2EndAvoidToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End, pickup3EndToPickup3, pickup3ToScore;

    public void buildPaths() {
        startToPickup0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup0Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup0Pose.getHeading())
                .build();

        pickup0ToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup0Pose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup0Pose.getHeading(), pickup1Pose.getHeading())
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

    private void launch() {
        catapultMode = CatapultModes.UP;
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
        catapultUpTimer.resetTimer();
    }

    private void updateCatapult() {
        if (catapultMode == CatapultModes.UP && catapultUpTimer.getElapsedTimeSeconds() > 0.5) {
            catapultMode = CatapultModes.DOWN;
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
            catapultDownTimer.resetTimer();
        } else if (catapultMode == CatapultModes.DOWN && catapultDownTimer.getElapsedTimeSeconds() > 0.15) {
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
                follower.followPath(startToPickup0, true);
                intake.setPower(INTAKE_IN_POWER);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()) {
                    launch();
                    setPathState(2);
                }
                break;

            case 2:
                if(isCatapultReady()) {
                    launch();
                    setPathState(3);
                }
                break;

            case 3:
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
                if(!follower.isBusy()) {
                    follower.followPath(pickup2EndToPickup2EndAvoid, true);
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(pickup2EndAvoidToScore, true);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()) {
                    launch();
                    setPathState(12);
                }
                break;

            case 12:
                if(isCatapultReady()) {
                    follower.followPath(scoreToPickup3, true);
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(pickup3ToPickup3End, true);
                    setPathState(14);
                }
                break;

            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(pickup3EndToPickup3, true);
                    setPathState(15);
                }
                break;

            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(pickup3ToScore, true);
                    setPathState(16);
                }
                break;

            case 16:
                if(!follower.isBusy()) {
                    launch();
                    setPathState(17);
                }
                break;

            case 17:
                if(isCatapultReady()) {
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        catapultUpTimer = new Timer();
        catapultDownTimer = new Timer();
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
        RED_TELEOP.startingPose = follower.getPose();
    }
}
