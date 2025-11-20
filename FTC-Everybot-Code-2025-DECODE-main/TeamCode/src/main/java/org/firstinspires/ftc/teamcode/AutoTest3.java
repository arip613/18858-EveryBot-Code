package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "SimplifiedAuto", group = "Examples")
public class AutoTest3 extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, catapultTimer;
    private TelemetryManager telemetryM;

    private DcMotor intake;
    private DcMotor catapult1, catapult2;

    private final double INTAKE_POWER = 1.0;
    private final double CATAPULT_UP_POWER = -1.0;
    private final double CATAPULT_DOWN_POWER = 1.0;
    private final double CATAPULT_HOLD_POWER = 0.2;

    private enum CatapultState { IDLE, LOADING, FIRING, HOLDING }
    private CatapultState catapultState = CatapultState.IDLE;

    private int pathState = 0;

    // Define poses
    private final Pose startPose = new Pose(132, 132, Math.toRadians(50));
    private final Pose pickup0Pose = new Pose(108, 108, Math.toRadians(50));
    private final Pose pickup1Pose = new Pose(108, 84, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(144, 84, Math.toRadians(0));
    private final Pose scorePose = new Pose(132, 132, Math.toRadians(50));
    private final Pose pickup2Pose = new Pose(108, 60, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(144, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(108, 36, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(144, 36, Math.toRadians(0));

    private PathChain startToPickup0, pickup0ToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup2, pickup2ToPickup2End, pickup2EndToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End, pickup3EndToScore;

    private void startLoading() {
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
        catapultState = CatapultState.LOADING;
    }

    private void startFiring() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
        catapultTimer.reset();
        catapultState = CatapultState.FIRING;
    }

    private void startHolding() {
        catapult1.setPower(CATAPULT_HOLD_POWER);
        catapult2.setPower(CATAPULT_HOLD_POWER);
        catapultState = CatapultState.HOLDING;
    }

    private void updateCatapult() {
        switch (catapultState) {
            case FIRING:
                if (catapultTimer.seconds() >= 0.3) { // Firing duration (simulates 0.3s trigger press)
                    startHolding();
                }
                break;
            case LOADING:
                // Keep loading until we transition to holding
                if (catapultTimer.seconds() >= 1.0) { // Auto transition to holding after 1 second
                    startHolding();
                }
                break;
            case HOLDING:
                // Maintain hold power
                catapult1.setPower(CATAPULT_HOLD_POWER);
                catapult2.setPower(CATAPULT_HOLD_POWER);
                break;
        }
    }

    private void buildPaths() {
        startToPickup0 = follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, pickup0Pose))
                .build();
        pickup0ToPickup1 = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup0Pose, pickup1Pose))
                .build();
        pickup1ToPickup1End = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setVelocityConstraint(5)
                .build();
        pickup1EndToScore = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, scorePose))
                .build();
        scoreToPickup2 = follower
                .pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .build();
        pickup2ToPickup2End = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                .setVelocityConstraint(5)
                .build();
        pickup2EndToScore = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, scorePose))
                .build();
        scoreToPickup3 = follower
                .pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .build();
        pickup3ToPickup3End = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3EndPose))
                .setVelocityConstraint(5)
                .build();
        pickup3EndToScore = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, scorePose))
                .build();
    }

    private void nextPathState() {
        pathState++;
        pathTimer.reset();
    }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        catapultTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");

        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        pathState = 0;
        pathTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();
        updateCatapult();
        intake.setPower(INTAKE_POWER);

        switch (pathState) {
            case 0: // Start (Score) -> Pickup0, begin loading
                follower.followPath(startToPickup0, true);
                startLoading();
                catapultTimer.reset();
                nextPathState();
                break;

            case 1: // Wait for path to complete
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 2: // At Pickup0 -> Fire catapult
                startFiring();
                nextPathState();
                break;

            case 3: // Wait 0.5 seconds after firing
                if (pathTimer.seconds() >= 0.5) {
                    nextPathState();
                }
                break;

            case 4: // Pickup0 -> Pickup1, start loading
                follower.followPath(pickup0ToPickup1, true);
                startLoading();
                catapultTimer.reset();
                nextPathState();
                break;

            case 5: // Wait for path to complete (should be in holding)
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 6: // Pickup1 -> Pickup1End (still in holding)
                follower.followPath(pickup1ToPickup1End, true);
                nextPathState();
                break;

            case 7: // Wait for path to complete
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 8: // Pickup1End -> Score (still in holding)
                follower.followPath(pickup1EndToScore, true);
                nextPathState();
                break;

            case 9: // Wait for path to complete
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 10: // At Score -> Fire catapult
                startFiring();
                nextPathState();
                break;

            case 11: // Wait 0.5 seconds after firing
                if (pathTimer.seconds() >= 0.5) {
                    nextPathState();
                }
                break;

            case 12: // Score -> Pickup2, start loading
                follower.followPath(scoreToPickup2, true);
                startLoading();
                catapultTimer.reset();
                nextPathState();
                break;

            case 13: // Wait for path to complete (should be in holding)
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 14: // Pickup2 -> Pickup2End (still in holding)
                follower.followPath(pickup2ToPickup2End, true);
                nextPathState();
                break;

            case 15: // Wait for path to complete
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 16: // Pickup2End -> Score (still in holding)
                follower.followPath(pickup2EndToScore, true);
                nextPathState();
                break;

            case 17: // Wait for path to complete
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 18: // At Score -> Fire catapult
                startFiring();
                nextPathState();
                break;

            case 19: // Wait 0.5 seconds after firing
                if (pathTimer.seconds() >= 0.5) {
                    nextPathState();
                }
                break;

            case 20: // Score -> Pickup3, start loading
                follower.followPath(scoreToPickup3, true);
                startLoading();
                catapultTimer.reset();
                nextPathState();
                break;

            case 21: // Wait for path to complete (should be in holding)
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 22: // Pickup3 -> Pickup3End (still in holding)
                follower.followPath(pickup3ToPickup3End, true);
                nextPathState();
                break;

            case 23: // Wait for path to complete
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 24: // Pickup3End -> Score (still in holding)
                follower.followPath(pickup3EndToScore, true);
                nextPathState();
                break;

            case 25: // Wait for path to complete
                if (!follower.isBusy()) {
                    nextPathState();
                }
                break;

            case 26: // At Score -> Fire catapult (final)
                startFiring();
                nextPathState();
                break;

            case 27: // Wait 0.5 seconds after final firing
                if (pathTimer.seconds() >= 0.5) {
                    nextPathState();
                }
                break;

            case 28: // Auto done
                intake.setPower(0);
                catapult1.setPower(0);
                catapult2.setPower(0);
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("Catapult State", catapultState);
        telemetry.addData("Path Timer", "%.2f", pathTimer.seconds());
        telemetry.addData("Catapult Timer", "%.2f", catapultTimer.seconds());
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("pathState", pathState);
        telemetryM.debug("catapultState", catapultState);
    }

    @Override
    public void stop() {
        intake.setPower(0);
        catapult1.setPower(0);
        catapult2.setPower(0);

        TeleOp4.startingPose = follower.getPose();

        telemetry.addData("Status", "Auto Stopped - Pose saved for TeleOp");
        telemetry.update();
    }
}