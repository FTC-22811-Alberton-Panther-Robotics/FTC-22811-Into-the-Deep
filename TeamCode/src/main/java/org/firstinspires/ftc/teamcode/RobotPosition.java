package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotPosition {
    private double x;
    private double y;
    private double heading;
    // Define a constructor that allows the OpMode to pass a reference to itself.

    public RobotPosition(OpMode opmode) {
        myOpMode = opmode;
    }
    // ... other methods
    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public void updatePosition(double leftEncoderDistance, double rightEncoderDistance, double frontEncoderDistance, double backEncoderDistance) {
        // Calculate average forward/backward and sideways distances
        double avgForwardBack = (frontEncoderDistance + backEncoderDistance) / 2;
        double avgSideways = (leftEncoderDistance + rightEncoderDistance) / 2;

        // Calculate change in heading based on wheel differences
        int WHEEL_BASE_WIDTH = 46; // mm
        double deltaHeading = (rightEncoderDistance - leftEncoderDistance) / WHEEL_BASE_WIDTH;

        // Update heading
        heading += deltaHeading;

        // Calculate change in x and y coordinates
        double deltaX = avgForwardBack * Math.cos(heading) - avgSideways * Math.sin(heading);
        double deltaY = avgForwardBack * Math.sin(heading) + avgSideways * Math.cos(heading);

        // Update position
        x += deltaX;
        y += deltaY;
    }
}