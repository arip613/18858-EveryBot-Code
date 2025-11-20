package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto", group = "Examples")
public class TestAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private TelemetryManager telemetryM;

    // Intake motor
    private DcMotor intake = null;
    private double INTAKE_IN_POWER = 1.0;

    private int pathState;
    private final Pose startPose = new Pose(132, 132, Math.toRadians(50)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(132, 132, Math.toRadians(50)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup0Pose = new Pose(108, 108, Math.toRadians(50)); //

    // Pickup 1 poses
    private final Pose pickup1Pose = new Pose(108, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1EndPose = new Pose(144, 84, Math.toRadians(0)); // End position after picking up first artifact

    // Pickup 2 poses
    private final Pose pickup2Pose = new Pose(108, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2EndPose = new Pose(144, 60, Math.toRadians(0)); // End position after picking up second artifact

    // Pickup 3 poses
    private final Pose pickup3Pose = new Pose(108, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3EndPose = new Pose(144, 36, Math.toRadians(0)); // End position after picking up third artifact

    // Path declarations
    private PathChain startToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup2, pickup2ToPickup2End, pickup2EndToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End, pickup3EndToScore;


    public void buildPaths() {
        // PICKUP 1 SEQUENCE
        startToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        pickup1ToPickup1End = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
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
                .build();

        pickup3EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start -> Pickup 1
                follower.followPath(startToPickup1, true);
                setPathState(1);
                break;
            case 1:
                // Wait at Pickup 1, then move to Pickup 1 End
                if(!follower.isBusy()) {
                    /* Grab artifact at pickup1 */
                    follower.followPath(pickup1ToPickup1End, true);
                    setPathState(2);
                }
                break;
            case 2:
                // Pickup 1 End -> Score
                if(!follower.isBusy()) {
                    /* Move to score with artifact */
                    follower.followPath(pickup1EndToScore, true);
                    setPathState(3);
                }
                break;
            case 3:
                // Score -> Pickup 2
                if(!follower.isBusy()) {
                    /* Score artifact */
                    follower.followPath(scoreToPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                // Wait at Pickup 2, then move to Pickup 2 End
                if(!follower.isBusy()) {
                    /* Grab artifact at pickup2 */
                    follower.followPath(pickup2ToPickup2End, true);
                    setPathState(5);
                }
                break;
            case 5:
                // Pickup 2 End -> Score
                if(!follower.isBusy()) {
                    /* Move to score with artifact */
                    follower.followPath(pickup2EndToScore, true);
                    setPathState(6);
                }
                break;
            case 6:
                // Score -> Pickup 3
                if(!follower.isBusy()) {
                    /* Score artifact */
                    follower.followPath(scoreToPickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                // Wait at Pickup 3, then move to Pickup 3 End
                if(!follower.isBusy()) {
                    /* Grab artifact at pickup3 */
                    follower.followPath(pickup3ToPickup3End, true);
                    setPathState(8);
                }
                break;
            case 8:
                // Pickup 3 End -> Score
                if(!follower.isBusy()) {
                    /* Move to score with artifact */
                    follower.followPath(pickup3EndToScore, true);
                    setPathState(9);
                }
                break;
            case 9:
                // Final scoring, then end
                if(!follower.isBusy()) {
                    /* Score final artifact and finish */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        telemetryM.update();

        // Keep intake running continuously
        intake.setPower(INTAKE_IN_POWER);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();

        // Panels telemetry for position
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("path state", pathState);
    }

    /** This method is called once at the init of the OpMode. **/
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
        TeleOp1.startingPose = follower.getPose();

    }
}