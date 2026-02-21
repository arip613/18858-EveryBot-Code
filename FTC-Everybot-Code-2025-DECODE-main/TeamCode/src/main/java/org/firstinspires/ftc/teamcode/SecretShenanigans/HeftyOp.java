package org.firstinspires.ftc.teamcode.SecretShenanigans;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * HeftyOp - Smooth Circle Demo using JController
 * Now with continuous path - NO MORE CHOPPY MOVEMENT!
 */
@Disabled
public class HeftyOp extends OpMode {

    private Follower follower;
    private JController robot;

    // Hardware
    private DcMotor intake;
    private DcMotor catapult1;
    private DcMotor catapult2;
    private Servo foot1;
    private Servo foot2;

    // Circle parameters
    private static final double CIRCLE_RADIUS = 30.0; // inches (increased for smoother motion)
    private static final int NUM_POINTS = 16; // More points = smoother circle

    // Starting position
    private Pose startingPose = new Pose(60, 60, 0);

    @Override
    public void init() {
        // Initialize hardware
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");
        foot1 = hardwareMap.get(Servo.class, "foot1");
        foot2 = hardwareMap.get(Servo.class, "foot2");

        // Set directions
        intake.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        foot1.setDirection(Servo.Direction.FORWARD);
        foot2.setDirection(Servo.Direction.REVERSE);

        // Set zero power behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Create JController
        robot = new JController(follower, intake, catapult1, catapult2, foot1, foot2);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Circle Radius", "%.1f inches", CIRCLE_RADIUS);
        telemetry.addData("Number of Points", NUM_POINTS);
        telemetry.addData("Motion", "SMOOTH!");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        // Build smooth continuous circle path using JController - NO PEDRO PATHING NEEDED!
        robot.StartPath()
                .Circle(
                        startingPose.getX(),  // Center X
                        startingPose.getY(),  // Center Y
                        CIRCLE_RADIUS,        // Radius
                        NUM_POINTS,           // Number of points
                        false                 // Face tangent (direction of travel)
                )
                .WithTangentHeading()         // Always face direction we're moving
                .Go();                        // Execute the path

        telemetry.addData("Status", "Running - SMOOTH Circle!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Just update - path runs continuously!
        robot.Update();

        // Telemetry
        telemetry.addData("Status", "Spinning SMOOTHLY!");
        telemetry.addData("Is Navigating", robot.IsNavigating());
        telemetry.addData("Distance to Target", "%.2f inches", robot.GetDistanceToTarget());
        telemetry.addData("Current X", "%.2f", robot.GetX());
        telemetry.addData("Current Y", "%.2f", robot.GetY());
        telemetry.addData("Current Heading", "%.1f°", robot.GetHeadingDegrees());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.StopAll();
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}