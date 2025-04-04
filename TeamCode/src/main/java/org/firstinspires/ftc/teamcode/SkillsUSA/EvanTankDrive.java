/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.SkillsUSA;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Evan Tank", group="SkillsUSA")
//@Disabled
public class EvanTankDrive extends OpMode{
    //Code for the webcam to work
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    /* Declare OpMode members. */
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;
    public Servo arm = null;

    public Servo claw = null;
    public Servo wrist = null;
    public static double armPosition = 0;
    public static double clawPosition = 0;
    public static double wristPosition = 0;

//    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
//    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
//    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //webcam stuff
        initWebcam();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        // Define and initialize ALL installed servos.
//        leftClaw  = hardwareMap.get(Servo.class, "left_hand");
//        rightClaw = hardwareMap.get(Servo.class, "right_hand");
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        double drive;
        double rot;
        double denominator;
        double Arm_power = .75;

        // Run  wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        drive = -gamepad1.left_stick_y;
        rot = gamepad1.left_stick_x * 0.5;
        denominator = Math.max(drive + rot,1);
        leftFrontDrive.setPower((drive + rot)/denominator);
        leftBackDrive.setPower((drive + rot)/denominator);
        rightFrontDrive.setPower((drive - rot)/denominator);
        rightBackDrive.setPower((drive - rot)/denominator);
//
// Use gamepad left & right Bumpers to open and close the claw
//        if (gamepad1.right_bumper)
//            clawOffset += CLAW_SPEED;
//        else if (gamepad1.left_bumper)
//            clawOffset -= CLAW_SPEED;
//
//        // Move both servos to new position.  Assume servos are mirror image of each other.
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//        leftClaw.setPosition(MID_SERVO + clawOffset);
//        rightClaw.setPosition(MID_SERVO - clawOffset);
//
        ///Green///

        // claw code
        clawPosition += (gamepad1.right_trigger - gamepad1.left_trigger) * 0.1;
        claw.setPosition(clawPosition);

        // Use gamepad buttons to move the arm up (Y) and down (A)
//        if (gamepad1.dpad_up)
//            arm.set*Position(Arm_power);
//        else if (gamepad1.dpad_down)
//            arm.setPosition(Arm_power);
//        else
//            arm.setPosition(0.0);
        //for the arm to move
        if (gamepad1.right_bumper)
            armPosition += 0.01;
        else if (gamepad1.left_bumper)
            armPosition -= 0.01;
        arm.setPosition(armPosition);


        //for the arm limmit
        if (armPosition< 0.0) {
            armPosition = 0.0;
        } else if (armPosition > 1.0){
            armPosition = 1.0;
        }
//
//        // Send telemetry message to signify robot running;
//        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
//        telemetry.addData("left",  "%.2f", left);
//        telemetry.addData("right", "%.2f", right);
//        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();
    }

    /**
         * Initialize the AprilTag processor.
         */
        private void initWebcam() {

            // Create the vision portal by using a builder.
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera).
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            // Choose a camera resolution. Not all cameras support all resolutions.
            //builder.setCameraResolution(new Size(640, 480));

            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            //builder.enableLiveView(true);

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            builder.setAutoStopLiveView(false);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();
        }
}
