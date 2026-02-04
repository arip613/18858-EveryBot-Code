package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Teleop.RED;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "LM5RED", group = "Auto")
public class RED12 extends OpMode {

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
    private final Pose startPose = new Pose(116.69051321928461, 132.29237947122863, Math.toRadians(37));
    private final Pose scorePose = new Pose(112.6750092686662, 122.45208942216487, Math.toRadians(40));
    private final Pose endPose = new Pose(115,68.5,Math.toRadians(0));

    private final Pose pickup1Pose = new Pose(89, 88.5, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(123.193, 88.5, Math.toRadians(0));
    private final Pose pickup1AvoidPose = new Pose(92.76049766718508, 66.14618973561431, Math.toRadians(0));
    //Note to Ari, fix positions.
    //private final Pose pickup2Pose = new Pose(20,20, Math.toRadians(0)); // Test Value to be removed.
    private final Pose pickup2Pose = new Pose(98,62, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(126, 62, Math.toRadians(0));
    private final Pose pickup2AvoidPose = new Pose(72, 84, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(89, 43, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(126, 43, Math.toRadians(0));

    private final Pose gateStartPose = new Pose(98, 67, Math.toRadians(0));
    private final Pose gateEndPose = new Pose(120.5, 69, Math.toRadians(0)); //19.5, 68, 0
    private final Pose gateAndIntakePose = new Pose(130.98911353032653, 61.44323483670298, Math.toRadians(15.67));
    private PathChain startToScore;
    private PathChain scoreToPickup2;
    private PathChain pickup2ToPickup2End;
    private PathChain pickup2EndToGateEnd;
    private PathChain gateEndToScore;
    private PathChain scoreToGateAndIntake;
    private PathChain gateAndIntakeToScore;
    private PathChain scoreToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End, pickup3ToScore, scoreToEndPose;
    private PrismAnimations.Solid solidBlue = new PrismAnimations.Solid(Color.BLUE);

    private GoBildaPrismDriver prism = null;



    public void buildPaths() {
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        scoreToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();


        pickup2ToPickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())
                .build();
        
        pickup2EndToGateEnd = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2EndPose, gateStartPose, gateEndPose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), gateStartPose.getHeading())
                .build();


        // Curved path from gateEnd through pickup1Avoid and pickup2Avoid to score
        gateEndToScore = follower.pathBuilder()
                .addPath(new BezierCurve(gateEndPose, pickup1AvoidPose, pickup2AvoidPose, scorePose))
                .setLinearHeadingInterpolation(gateEndPose.getHeading(), scorePose.getHeading())
                .build();

        // Curved path from score through pickup2Avoid to gateAndIntake
        scoreToGateAndIntake = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2AvoidPose, pickup1AvoidPose, gateAndIntakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateAndIntakePose.getHeading())
                .build();

        // Curved path from gateAndIntake through pickup1Avoid and pickup2Avoid to score
        gateAndIntakeToScore = follower.pathBuilder()
                .addPath(new BezierCurve(gateAndIntakePose, pickup1AvoidPose, pickup2AvoidPose, scorePose))
                .setLinearHeadingInterpolation(gateAndIntakePose.getHeading(), scorePose.getHeading())
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

        pickup3ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3EndPose, pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePose.getHeading())
                .build();
        scoreToPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2AvoidPose, pickup2Pose))
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
        } else if (catapultMode == CatapultModes.DOWN && catapultDownTimer.getElapsedTimeSeconds() > 0.4) {
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
                // Go to Score Position
                follower.followPath(startToScore, true);
                intake.setPower(INTAKE_IN_POWER);
                setPathState(1);
                waitTimer.resetTimer();
                break;

            case 1:
                //After waiting for 0.7 secs if NOT moving.
                if(!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 0.7) {
                    launch();
                    setPathState(2);
                }
                break;

            case 2:

                if(isCatapultReady()) {
                    intake.setPower(INTAKE_IN_POWER);
                    follower.followPath(scoreToPickup2, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(pickup2ToPickup2End, 0.75, true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(pickup2EndToGateEnd, 0.65, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    waitTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if(waitTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(gateEndToScore, true);
                    setPathState(7);
                    waitTimer.resetTimer();
                }
                break;

            case 7:
                if(!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 1.8) {
                    launch();
                    setPathState(8);
                    waitTimer.resetTimer();
                }
                break;

            case 8:
                if(isCatapultReady()) {
                    follower.followPath(scoreToPickup1, true);
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()) {
                    intake.setPower(INTAKE_IN_POWER);
                    follower.followPath(pickup1ToPickup1End, 0.75, true);
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(pickup1EndToScore, true);
                    setPathState(11);
                    waitTimer.resetTimer();
                }
                break;

            case 11:
                if(!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 0.5) {
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
                    follower.followPath(pickup3ToPickup3End, 0.75, true);
                    setPathState(14);
                }
                break;

            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(pickup3ToScore, true);
                    setPathState(15);
                    waitTimer.resetTimer();
                }
                break;
            case 15:
                if(!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 0.2) {
                    launch();
                    setPathState(16);
                }
                break;

            case 16:
                if(isCatapultReady()) {
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(scorePose, endPose))
                            .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                            .build(), true);
                    setPathState(17);
                }
                break;


            case 17:
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
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        solidBlue.setBrightness(100);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidBlue);
    }

    @Override
    public void init_loop() {
        RED.startingPose = follower.getPose();
    }


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
        RED.startingPose = follower.getPose();




    }
}