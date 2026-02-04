package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Foot Calibration", group = "Test")
public class FOOTCALIBRATION extends OpMode {

    private Servo foot1;
    private Servo foot2;

    private static final double STEP = 0.025;
    private double footPosition = 0.0;

    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

    @Override
    public void init() {
        foot1 = hardwareMap.get(Servo.class, "foot1");
        foot2 = hardwareMap.get(Servo.class, "foot2");

        foot1.setDirection(Servo.Direction.FORWARD);
        foot2.setDirection(Servo.Direction.REVERSE);

        footPosition = 0.0;
        foot1.setPosition(footPosition);
        foot2.setPosition(footPosition);
    }

    @Override
    public void loop() {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadUp && !wasDpadUpPressed) {
            footPosition += STEP;
        }

        if (dpadDown && !wasDpadDownPressed) {
            footPosition -= STEP;
        }

        footPosition = Math.max(0.0, Math.min(1.0, footPosition));

        foot1.setPosition(footPosition);
        foot2.setPosition(footPosition);

        wasDpadUpPressed = dpadUp;
        wasDpadDownPressed = dpadDown;

        telemetry.addData("Foot Position", "%.4f", footPosition);
        telemetry.addData("Foot1 Raw", "%.4f", foot1.getPosition());
        telemetry.addData("Foot2 Raw", "%.4f", foot2.getPosition());
        telemetry.addLine("DPAD UP: +0.025 | DPAD DOWN: -0.025");
        telemetry.update();
    }
}
