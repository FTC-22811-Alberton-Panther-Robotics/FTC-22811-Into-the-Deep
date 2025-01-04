package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called org.firstinspires.ftc.teamcode.RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy org.firstinspires.ftc.teamcode.MorrisPOVDrive.java into TeamCode (using Android Studio or OnBotJava) then org.firstinspires.ftc.teamcode.RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 */

/**
 * Mr. Morris:          TO DO:  1) Test, then revise code for arm, lift, gripper, and wrist
 *                              2) Write code and presets for lift. An automated hang sequence is especially needed.
 *                              3) Write code for wrist.
 *                              4) Once second lift motor is installed - Implement code in RobotHardware
 *                              5) Change code when active intakeLeft and color sensor is implemented
 *                              6) Consider implementing Driver-centric toggle
 *                              7) Set up code for hardware limit switches for arm and lift - Edit: this may not be necessary with the over current sensing
 *
 *     COMPLETE, NEEDS TESTING: 1) Finish updating to match RobotHardware file definitions, then delete or comment out @Disabled
 *                              3) Use math to keep wrist turned so that gripper is level with ground (rotate relative to arm rotation) - Edit: this is no longer necessary and is broken
 *                              4) Add elapsed time tracking and implement better button press delay method. See <a href="https://stemrobotics.cs.pdx.edu/node/7262.html">...</a>
 *                              5) Implemented a state machine method of tracking the arm movement with over current detection to prevent the arm from doing damage to itself. Check it out in the RobotHardware class.
 *                              6) Wrote code for arm presets and incrementation. Needs refinement once we have the lift scoring basket in place
 *                              7) Fixed initial offset code for arm
 */

@TeleOp(name="Morris POV Drive", group="Test Code")
@Disabled
public class MorrisPOVDrive extends LinearOpMode {

    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot       = new RobotHardware(this, telemetry);

    @Override
    public void runOpMode() {
        double forward;
        double turn;
        double strafe;
        double aLastTime = 0, bLastTime = 0, xLastTime = 0, yLastTime = 0, rBLastTime = 0, lBLastTime = 0, dPadUpLastTime = 0, dpadDownLastTime = 0;
        boolean aButtonPressed = false, bButtonPressed = false, xButtonPressed = false, yButtonPressed = false, dPadUpPressed = false, dPadDownPressed = false;
        final int ARM_MIN_ANGLE = 10, ARM_MAX_ANGLE = 130;
        final double BUTTON_PRESS_DELAY = .075;// seconds, keep track of how long a button has been pressed and allow for a quick press to move a servo a small amount while a long press moves the servo a longer distance.

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        ElapsedTime runtime = new ElapsedTime();


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play to start OpMode.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.opModeActive = true;
            robot.updateArmState(); // update arm state machine to track arm position. By calling it here it gets updated everytime the opMode loops but otherwise works in the background while motors move.

            // Run wheels in strafer mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back and left and right, the Right stick turns left and right.
            // This way it's easy to drive diagonally and have good control of heading.

            ////Mr. Morris: Alternatively we could use right trigger for forward, left trigger for reverse, left_stick_x for turning
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn  =  gamepad1.right_stick_x;

            // Combine forward and turn for blended motion. Use org.firstinspires.ftc.teamcode.RobotHardware class
            robot.mecanumDrive(forward, strafe, turn);

            // Use gamepad left & right Bumpers to open and close the gripper claw
            // Use the SERVO constants defined in org.firstinspires.ftc.teamcode.RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range

            // Close gripper when right bumper is pressed if it's not already at max, open gripper when left bumper is pressed if it's not already at min
            // Keeps track of how long a button is pressed and moves a small amount for a short press and a larger amount for a long press
//            if (gamepad1.right_bumper && runtime.seconds() - rBLastTime > BUTTON_PRESS_DELAY){
//                robot.gripperIncrement();
//                rBLastTime = runtime.seconds();
//            }
//            if (gamepad1.left_bumper && runtime.seconds() - lBLastTime > BUTTON_PRESS_DELAY){
//                robot.gripperDecrement();
//                lBLastTime = runtime.seconds();
//            }

            if (gamepad1.y){
                if (!yButtonPressed){
                    robot.setArmAngle(ARM_MAX_ANGLE);
                    yButtonPressed = true;
                }
            } else yButtonPressed = false;

            if (gamepad1.x){
                if (!xButtonPressed){
                    robot.setArmAngle(ARM_MIN_ANGLE);
                    xButtonPressed = true;
                }
            } else xButtonPressed = false;


            // Use gamepad buttons to rotate arm forward/down (dpad down) and back/up (dpad up)
            // Use the MOTOR constants defined in org.firstinspires.ftc.teamcode.RobotHardware class.
            /**
             * Mr. Morris: TO DO: Consider redefining the arm movements to use a joystick or triggers so that the analog variable power can be used to fine tune control,
             *              then use a,b,x,y buttons for preset positions like in last year's FTC season
             */
            if (gamepad1.b){
                if (!bButtonPressed){
                    robot.armAngleIncrement();
                    bButtonPressed = true;
                }
            } else bButtonPressed = false;

            if (gamepad1.a){
                if (!aButtonPressed){
                    robot.armAngleDecrement();
                    aButtonPressed = true;
                }
            } else aButtonPressed = false;

//            if (gamepad1.dpad_up) {
//                if (!dPadUpPressed) {
//                    robot.pIDIncrement += 1;
//                    dPadUpPressed = true;
//                }
//            } else dPadUpPressed = false;
//
//            if (gamepad1.dpad_down) {
//                if (!dPadDownPressed) {
//                    robot.pIDIncrement -= 1;
//                    dPadDownPressed = true;
//                }
//            } else dPadDownPressed = false;


            // Use gamepad buttons to extend lift (X) and retract lift (B)
            // Use the MOTOR constants defined in org.firstinspires.ftc.teamcode.RobotHardware class.
//            if (gamepad1.x && getRuntime() - xLastTime > BUTTON_PRESS_DELAY) {
//                if (liftExtendTargetInches < RobotHardware.LIFT_EXTEND_3STAGE_MAX_INCHES)
//                    liftExtendTargetInches += RobotHardware.LIFT_EXTEND_INCREMENT_INCHES; // extend when X is pressed
//                xLastTime = getRuntime();
//                }
//            else if (gamepad1.b && getRuntime() - bLastTime > BUTTON_PRESS_DELAY) {
//                if (liftExtendTargetInches > RobotHardware.LIFT_RETRACT_MAX_INCHES) {
//                    liftExtendTargetInches -= RobotHardware.LIFT_RETRACT_INCREMENT_INCHES; // retract when B is pressed
//                }
//                bLastTime = getRuntime();
//            }
//            else liftExtendTargetInches = 0; // don't move if x and b aren't pressed.
////            robot.setArmPosition(armRotateTarget); //send rotation values to robot
//            robot.setLiftPositionInches(liftExtendTargetInches); // send extension values to robot

            /** This code needs updated. We probably don't want the wrist matching the arm angle all the time since it needs to reach back to the lift */
            // Move wrist so that it moves when arm rotates to keep gripper parallel to floor
            // e.g. if arm angle is at -30 (30 degrees below forward horizontal), wrist must be 30 (30 degrees above forward horizontal) to keep gripper horizontal
//            robot.setWristAngle(-robot.getArmAngle());

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Arm Up/Down", "Y & A Buttons");
            telemetry.addData("Lift Extend/Retract", "X & B Buttons");
            telemetry.addData("Gripper Open/Closed", "Left and Right Bumpers");
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", forward);
            telemetry.addData("Turn Power",  "%.2f", turn);
//            telemetry.addData("Arm Extend Power",  "%.2f", armExtendTarget);
            telemetry.addData("Arm Angle Relative to Zero", "%.2f",robot.getArmAngleRelativeToZero());
            telemetry.addData("Arm Target Angle", "%.2f",robot.getArmTargetAngle());
            telemetry.addData("Arm Position", "%.2f",robot.getArmEncoderCounts());
            telemetry.addData("Arm Target Position", "%.2f",robot.getArmTargetPosition());
            telemetry.addData("Arm State", robot.getArmState());
            telemetry.addData("Arm Current (Amps)", robot.getArmCurrentAmps());
            telemetry.addData("Runtime", "%.2f",runtime.seconds());

            telemetry.update();

            idle(); //share processor with other programs - good to include in any loop structure in a linear OpMode
        }
        robot.opModeActive = false;
    }
}