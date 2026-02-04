package org.firstinspires.ftc.teamcode.Util;

public class VelocityShotController {
    private double targetX, targetY;
    private double calculatedHeading=0;

    public VelocityShotController(double targetX, double targetY, double unused) {
        this.targetX = targetX;
        this.targetY = targetY;
    }

    public double calculateHeading(double currentX, double currentY) {
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        calculatedHeading = Math.atan2(dy, dx);
        return calculatedHeading;
    }

}