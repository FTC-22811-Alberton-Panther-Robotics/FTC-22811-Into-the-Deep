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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

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



@TeleOp(name="Elijah Tank", group="SkillsUSA")
//@Disabled
public class ElijahTeleop extends OpMode {

    /* Declare OpMode members. */

    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;

    public DcMotor rightBack = null;

    public DcMotor leftBack = null;

    public DcMotor lowerArm = null;

    public DcMotor upperArm = null;

    public Servo claw = null;

    public Servo wrist = null;
    public static final int ARM_INCREMENT = 100;
    public static final int ARM_MAX = 1000;
    public static final int ARM_MIN = 0;
    public static final int SLIDE_INCREMENT = 100;
    public static final int SLIDE_MAX = 1000;
    public static final int SLIDE_MIN = 0;
    public static double wristPosition = 0;
    public static double clawPosition = 0;
    public static double upperArmPosition = 0;
    public static double lowerArmPosition = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        lowerArm = hardwareMap.get(DcMotor.class, "lowerarm");
        upperArm = hardwareMap.get(DcMotor.class, "upperarm");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //webcam stuff
        initWebcam();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
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
        rot = -gamepad1.left_stick_x;
        denominator = Math.max(drive + rot,1);
        leftFront.setPower((drive - rot)/denominator);
        leftBack.setPower((drive - rot)/denominator);
        rightFront.setPower((drive + rot)/denominator);
        rightBack.setPower((drive + rot)/denominator);

        // Claw code
        clawPosition += (gamepad1.right_trigger - gamepad1.left_trigger) * .03;
        claw.setPosition(clawPosition);
        //Claw limit
        if (clawPosition > 1)
            clawPosition = 1;
        else if (clawPosition < 0) {
            clawPosition = 0;
        }

        //Wrist code
        if (gamepad1.right_bumper) {
            wristPosition += 0.009;
        } else if (gamepad1.left_bumper) {
            wristPosition -= 0.009
            ;
        }
        //Wrist limit
        if (wristPosition > 1)
            wristPosition = 1;
        else if (wristPosition < 0) {
            wristPosition = 0;
        }
        wrist.setPosition(wristPosition);

        // Use gamepad buttons to move the arm up (Y) and down (A)

        lowerArm.setTargetPosition(lowerArm.getCurrentPosition() - (int) (gamepad1.right_stick_y * ARM_INCREMENT));
        upperArm.setTargetPosition(upperArm.getCurrentPosition() + (int) (gamepad1.right_stick_x * SLIDE_INCREMENT));
        lowerArm.setPower(1);
      //
        //
        lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperArm.setPower(1);
       upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        if (lift_motor.getCurrentPosition() > ARM_MAX)
//            lift_motor.setTargetPosition((int) ARM_MAX);
//        else if (lift_motor.getCurrentPosition() < ARM_MIN)
//            lift_motor.setTargetPosition((int) ARM_MIN);
//
//        if (slide_motor.getCurrentPosition() > SLIDE_MAX)
//            slide_motor.setTargetPosition((int) SLIDE_MAX);
//        else if (slide_motor.getCurrentPosition() < SLIDE_MI6N)
//            slide_motor.setTargetPosition((int) SLIDE_MIN);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw position: ", clawPosition);
        telemetry.addData("wrist position: ", wristPosition);
        telemetry.addData("lower arm position: ", lowerArm.getCurrentPosition());
        telemetry.addData("upper arm position: ", upperArm.getCurrentPosition());
        telemetry.addData("drive", "%.2f", drive);
        telemetry.addData("right", "%.2f", rot);
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

