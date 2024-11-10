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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode is based on the RobotAutoDriveByTime_Linear sample code.
 * I have adapted it to utilize the RobotHardware class.
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Strafe right for 1 second
 *   - Drive diagonally to left for 2 seconds
 *   - Drive Backward for 1 Second
 *   - Drive in an arc to the left for 4 seconds
 *   - Stop
 *
 */

@Autonomous(name="Robot: Auto Drive By Time", group="Test Code", preselectTeleOp = "MorrisPOVDrive")
//@Disabled
public class MorrisAutoDriveByTime extends LinearOpMode {

    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    /* Declare OpMode members. */
    int legNumber = 0;
    boolean isRedAlliance = true;
    boolean isNearStart = true;
    boolean option3 = true;
    double startingPause = 0;

    @Override
    public void runOpMode() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Select Alliance, starting point, and route options
        configAutonomous();

        // Wait for the game to start (driver presses START)
        waitForStart();

        ElapsedTime runTime = new ElapsedTime();
        while(runTime.seconds() < startingPause); // do nothing during starting pause period

        if(isRedAlliance && isNearStart) {
            // Drive forward for 3 seconds
            driveForTime(3, .6, 0, 0);

            // Spin right for 1.3 seconds
            driveForTime(1.3, 0, .5, 0);

            // Stop
            driveForTime(1, 0, 0, 0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
        if(!isRedAlliance && isNearStart) {
            // Strafe right for 1 second
            driveForTime(1, 0, 0, .5);

            // Drive diagonally to left for 2 seconds
            driveForTime(2, .3, 0, .3);

            // Stop
            driveForTime(1, 0, 0, 0);

        }
        if(isRedAlliance && !isNearStart){
            // Drive backward for 1 second
            driveForTime(1, -1, 0, 0);

            // Drive in a tight arc to the left for 4 seconds
            driveForTime(4, .5, -1, 0);

            // Stop
            driveForTime(1, 0, 0, 0);
        }
        if(!isRedAlliance && !isNearStart){
            // Stop
            driveForTime(1, 0, 0, 0);
        }
    telemetry.addData("Path", "Complete");
    telemetry.update();
    sleep(1000);
    }

    /**
    * @param time       Time in seconds to complete the action.
    * @param forward     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
    * @param turn       Turn speed (-1.0 to 1.0) +ve is right
    * @param strafe     Right/Left strafing (-1.0 to 1.0) +ve is right
    * This function is used for just turning.
    */
    private void driveForTime(double time, double forward, double turn, double strafe) {
        legNumber += 1;
        robot.mechanumDrive(forward, strafe, turn);
        ElapsedTime legTime = new ElapsedTime();
        while (opModeIsActive() && (legTime.seconds() < time)) {
            telemetry.addData("Path", "Leg ", legNumber, ": %4.1f S Elapsed", legTime.seconds());
            telemetry.update();
        }
    }


    /**
     * This method used a technique called latching with all the button presses where it only toggles
     * when you let go of the button. This keeps from accidentally registering multiple button presses
     * when you hold the button for too long.
     */
    private void configAutonomous(){
        int selection = 0;
        final int SELECTION_COUNT = 4; // Number of options in selection list.
        boolean dpadDownPressed = false;
        boolean dpadUpPressed = false;
        boolean dpadRightOrLeftPressed = false;
        boolean configComplete = false;

        // This loops until the x
        while(!isStarted() && !configComplete){
            // This is the first example of latching. This while loop can cycle hundreds of times a
            // second. This waits until the dpad_down button is not pressed before it increments
            // the selection
            if(gamepad1.dpad_down) dpadDownPressed = true;
            else if(dpadDownPressed && !gamepad1.dpad_down){
                dpadDownPressed = false;
                selection += 1;
                selection = selection % SELECTION_COUNT; // cycles around to beginning of list after the end
                                                         // % means the remainder after dividing
            }

            if(gamepad1.dpad_up) dpadUpPressed = true;
            else if(dpadUpPressed && !gamepad1.dpad_up){
                dpadUpPressed = false;
                if (selection >0) selection -= 1;
            }

            // This block displays an arrow next to the option currently being selected and waits for
            // you to toggle that option by pressing either dpad_left or dpad_right
            if(selection == 1) {
                telemetry.addData("-->Alliance: ", isRedAlliance ?"Red": "Blue"); //This syntax is an inline if statement that can be used in simple cases
                if(gamepad1.dpad_right || gamepad1.dpad_left) dpadRightOrLeftPressed = true;
                else if(dpadRightOrLeftPressed && !(gamepad1.dpad_right || gamepad1.dpad_left)){
                    dpadRightOrLeftPressed = false;
                    isRedAlliance = !isRedAlliance;
                }
            } else telemetry.addData("Alliance: ", isRedAlliance ?"Red": "Blue");

            if(selection == 2) {
                telemetry.addData("-->Starting Position: ", isNearStart ?"Near": "Far");
                if(gamepad1.dpad_right || gamepad1.dpad_left) dpadRightOrLeftPressed = true;
                else if(dpadRightOrLeftPressed && !(gamepad1.dpad_right || gamepad1.dpad_left)){
                    dpadRightOrLeftPressed = false;
                    isNearStart = !isNearStart;
                }
            } else telemetry.addData("Starting Position: ", isNearStart ?"Near": "Far");

            if(selection == 3) {
                telemetry.addData("-->Option 3: ", option3 ?"True": "False");
                if(gamepad1.dpad_right || gamepad1.dpad_left) dpadRightOrLeftPressed = true;
                else if(dpadRightOrLeftPressed && !(gamepad1.dpad_right || gamepad1.dpad_left)){
                    dpadRightOrLeftPressed = false;
                    isNearStart = !isNearStart;
                }
            } else telemetry.addData("Option 3: ", option3 ?"True": "False");

            if(selection == 4) {
                telemetry.addData("-->Starting Pause: ", startingPause +" seconds");
                if(gamepad1.dpad_right) dpadRightOrLeftPressed = true;
                else if(dpadRightOrLeftPressed && !gamepad1.dpad_right){
                    dpadRightOrLeftPressed = false;
                    startingPause += .5;
                }
                if(gamepad1.dpad_left) dpadRightOrLeftPressed = true;
                else if(dpadRightOrLeftPressed && !gamepad1.dpad_left){
                    dpadRightOrLeftPressed = false;
                    startingPause -= .5;
                }
            } else telemetry.addData("Starting Pause: ", startingPause +" seconds");

            telemetry.addLine("\nPress [x] to complete");

            // Press x to end Autonomous Configuration
            if(gamepad1.x) configComplete = true;
            telemetry.update();
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Autonomous Configuration Complete.  Press Play to start OpMode.");
        telemetry.update();
    }



}