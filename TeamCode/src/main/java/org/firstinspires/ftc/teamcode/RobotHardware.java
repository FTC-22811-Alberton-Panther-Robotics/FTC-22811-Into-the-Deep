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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This file works in conjunction with the External Hardware Class sample called: org.firstinspires.ftc.teamcode.MorrisPOVDrive.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes six motors (left_front_drive, left_rear_drive, right_front_drive, right_rear_drive, arm_rotate, and arm_extend) and two servos (wrist and gripper)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named org.firstinspires.ftc.teamcode.RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample org.firstinspires.ftc.teamcode.MorrisPOVDrive.java, and select TeleOp.
 *
 */

////Mr. Morris made changes to have 4 drive motors, arm rotation and extension, and wrist and gripper servos

public class RobotHardware {

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.


    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive   = null;
    private DcMotor leftRearDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor rightRearDrive  = null;
    private DcMotor arm = null;
    private DcMotor lift = null;
    private DcMotorEx armEx = null;
    private Servo   wrist = null;
    private Servo   gripper = null;
    private Telemetry telemetry;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REVOL    = 537.7 ;    // GoBilda 312rpm motor has this encoder resolution
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // No Gear Reduction.
    static final double     WHEEL_DIAMETER_INCHES   = 96 * 25.4 ;     // 96mm converted to inches. For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REVOL * DRIVE_GEAR_REDUCTION) /
                                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     MAX_CURRENT_AMPS        = 6;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    /** Mr. Morris: TO DO: test and update servo speeds. */
    public static final double GRIPPER_INCREMENT = 0.06, GRIPPER_MAX = 1, GRIPPER_MIN = 0 ;  // sets rate to move gripper servo and max and min travel. If you use SRS servo programmer to set limits, this will be 1 and 0. If you need to limit travel in the software, this is where to do it.
    public static final double WRIST_INCREMENT = 0.02 ; // sets rate to move wrist servo
    public static final double WRIST_MAX_ANGLE  = 300 ; // Adjust this angle if SRS servo programmer has limited servo travel to less than 300
    public static final int ARM_INCREMENT_DEGREES = 10, ARM_ROTATE_MAX = 160, ARM_ROTATE_MIN = -20, // Straight forward defined as 0 degrees
                            ARM_ROTATE_ENCODER_RESOLUTION = 28, ARM_ROTATE_GEAR_RATIO = 60, ARM_STARTING_ANGLE_OFFSET = 160;
    public static final int LIFT_ENCODER_RESOLUTION = 28, LIFT_GEAR_RATIO = 20;
    public static final double LIFT_TRAVEL_PER_ROTATION_INCHES = 120 / 25.4, LIFT_EXTEND_INCREMENT_INCHES = 0.50, LIFT_RETRACT_INCREMENT_INCHES = -0.50, LIFT_EXTEND_3STAGE_MAX_INCHES = 732 / 25.4,
            LIFT_RETRACT_MAX_INCHES = 0;
    private int targetPosition = 0;
    ElapsedTime stateTimer = new ElapsedTime();
    private static final long ARM_POSITION_TIMEOUT = 2000;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(OpMode opmode, Telemetry telemetry) {
        this.telemetry = telemetry;
        myOpMode = opmode;
    }

    // Create a state machine to track what state the arm motor is in. A state machine is a computational model that represents a system
    // with a finite number of states and transitions between those states. It's a powerful tool for managing complex logic and behavior,
    // especially in systems with dynamic or event-driven interactions. Check out the updateArmState function to see how this works.
    private enum ArmState { IDLE, MOVING_TO_TARGET, HOLDING_POSITION, STALLED, TIMEOUT, ERROR}
    private ArmState armCurrentState = ArmState.IDLE;
    public boolean holdArm = false; // consider triggering the hold behavior in the opMode with a button press TRUE: maintain arm power after RUN_TO_POSITION is complete, FALSE: stop arm power

    public double armTargetAngleDegrees = 0;
    public int liftExtendTargetInches = 0;
    public boolean opModeActive = false;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive  = myOpMode.hardwareMap.get(DcMotor.class, "right_rear_drive");
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        lift   = myOpMode.hardwareMap.get(DcMotor.class, "lift");

        // Take advantage of the extended features of the DcMotorEx class, such as reading current draw and setting velocity, if the motor supports it
        if (arm instanceof DcMotorEx) {
            armEx = (DcMotorEx) arm;
        } else telemetry.addData("WARNING: ", "Arm is not a DcMotorEx");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here reflect our belt drive wheels. 90degree miter gear driven wheels need the directions flipped.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders at start of program
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Since there are encoders connected, RUN_USING_ENCODER mode is enabled for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        gripper = myOpMode.hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(GRIPPER_MIN);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the motor powers required to achieve the requested
     * robot motions: forward (Axial motion) strafe (Lateral motion) and turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param forward     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Right/Left strafing (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void mechanumDrive(double forward, double strafe, double turn) {
        // Combine forward, strafe, and turn for blended motion.
        double leftFront  = forward + strafe + turn;
        double leftRear = forward - strafe + turn;
        double rightFront = forward - strafe - turn;
        double rightRear = forward + strafe - turn;

        // Scale the values so none of them exceed +/- 1.0
        double max1 = Math.max(Math.abs(leftFront),Math.abs(leftRear));
        double max2 = Math.max(Math.abs(rightFront), Math.abs(rightRear));
        double max = Math.max(Math.abs(max1), Math.abs(max2));
        if (max > 1.0)
        {
            leftFront /= max;
            leftRear /= max;
            rightFront /= max;
            rightRear /= max;
        }

        // with the new scaled values, send power to the motors.
        setDrivePower(leftFront, leftRear, rightFront, rightRear);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed, double forwardInches, double rightInches, double rotateAngle){

    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * Mr. Morris: Our robot is 4 wheel drive, but in tank drive configuration the two left wheels
     *             always travel at the same speed and the two right wheels travel at the same speed.
     *
     * @param leftFront     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftRear      Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFront    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightRear     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */

    public void setDrivePower(double leftFront, double leftRear, double rightFront, double rightRear) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFront);
        leftRearDrive.setPower(leftRear);
        rightFrontDrive.setPower(rightFront);
        rightRearDrive.setPower(rightRear);
    }

    public int[] getDriveEncoderValues(){
        return new int[]{leftFrontDrive.getCurrentPosition(), leftRearDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition()};
    }

    // Function to calculate encoder position from target angle
    private int calculateArmEncoderValue(double armAngle) {
        int ticksPerRevolution = ARM_ROTATE_ENCODER_RESOLUTION * ARM_ROTATE_GEAR_RATIO;
        double ticksPerDegree = (double) ticksPerRevolution / 360;
        return (int) (armAngle * ticksPerDegree);
    }

    // Function to calculate arm angle from encoder position
    private double calculateAngleFromEncoderValue(int encoderPosition) {
        int ticksPerRevolution = ARM_ROTATE_ENCODER_RESOLUTION * ARM_ROTATE_GEAR_RATIO;
        double degreesPerTick = 360.0 / ticksPerRevolution;
        return encoderPosition * degreesPerTick;
    }

    public void armAngleIncrement(){
        if (getArmAngleRelativeToZero() < ARM_ROTATE_MAX) {
            setArmAngle(getArmAngleAbsolute() + ARM_INCREMENT_DEGREES);
        }
    }

    public void armAngleDecrement(){
        if (getArmAngleRelativeToZero() > ARM_ROTATE_MIN){
            setArmAngle((getArmAngleAbsolute() - ARM_INCREMENT_DEGREES));
        }
    }

    /**
     * Get the encoder information for the arm rotation motor and convert it to degrees.
     * Mr. Morris: TO DO: Might need to initialize arm angle on startup and/or adjust for starting/resting position.
     *                    i.e. if starting location is 160 degrees, initialize and offset it at program start to account for this.
     *                    Add telemetry statement to test and adjust this.
     */
    public double getArmAngleRelativeToZero(){
        int encoderCounts = arm.getCurrentPosition();
        double angleRelativeToZero = calculateAngleFromEncoderValue(encoderCounts) + ARM_STARTING_ANGLE_OFFSET;
        return angleRelativeToZero;
    }

    public double getArmAngleAbsolute(){
        return calculateAngleFromEncoderValue(arm.getCurrentPosition());
    }

    public double getArmEncoderCounts(){
        return arm.getCurrentPosition();
    }

    /**
     * Given a position to go to in inches, convert to encoder values and set the lift position
     * Change motor direction depending on if it is above or below the desired position
     * @param targetPositionInches is how high the lift needs to be set to
     * Need to account for its starting height
     */
    public void setLiftPositionInches(int targetPositionInches) {
        liftExtendTargetInches = targetPositionInches;
        lift.setTargetPosition(calculateEncoderPositionFromLiftInches(targetPositionInches));
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(.3); // adjust power as needed

        while (opModeActive && arm.isBusy()){
            // Use telemetry to monitor the motor's status
            telemetry.addData("Lift Target Position", "%d",calculateEncoderPositionFromLiftInches(targetPositionInches));
            telemetry.addData("Lift Current Position", "%d",lift.getCurrentPosition());
            telemetry.update();
            Thread.yield();
        }

        // Stop the motor when it reaches the target position
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private int calculateLiftInchesFromEncoderPosition(int encoderPosition) {
        int ticksPerRevolution = LIFT_ENCODER_RESOLUTION * LIFT_GEAR_RATIO;
        double inchesPerTick = LIFT_TRAVEL_PER_ROTATION_INCHES / ticksPerRevolution;
        return (int) (encoderPosition * inchesPerTick);
    }

    private int calculateEncoderPositionFromLiftInches(double liftInches) {
        int ticksPerRevolution = LIFT_ENCODER_RESOLUTION * LIFT_GEAR_RATIO;
        double inchesPerTick = LIFT_TRAVEL_PER_ROTATION_INCHES / ticksPerRevolution;
        return (int) (liftInches / inchesPerTick);
    }

    public double getLiftExtensionInches(){
        return calculateLiftInchesFromEncoderPosition(lift.getCurrentPosition());
    }

    /**
     * Mr. Morris: TO DO: may want to work in degrees, then convert to range from -0.5 to 0.5, see setWristAngle() function
     * Send the gripper the new position to go to
     * @param position value from 0 to 1
     */
    public void setGripperPosition(double position) {
        gripper.setPosition(position);
    }

    public void gripperIncrement(){
        if (gripper.getPosition() < GRIPPER_MAX) {
            setGripperPosition(gripper.getPosition() + GRIPPER_INCREMENT);
        }
    }

    public void gripperDecrement(){
        if (gripper.getPosition() > GRIPPER_MIN){
            setGripperPosition((gripper.getPosition() - GRIPPER_INCREMENT));
        }
    }

    public double getGripperPosition(){
        return gripper.getPosition();
    }

    /**
     * Send the wrist to a certain angle in degrees
     * @param position is a number from 0 to 1
     */
    public void setWrist(double position){
        //write code to set wrist here
    }

    // The following function creates a state machine. A state machine is a computational model that represents a system with a finite number of states and transitions between
    // those states. It's a powerful tool for managing complex logic and behavior, especially in systems with dynamic or event-driven interactions.
    // Key Components:
    // States: Represent the different possible conditions or modes of the system (e.g., "idle," "moving," "holding," "error").
    // Transitions: Define the rules for moving from one state to another based on events or conditions (e.g., sensor readings, user input, timeouts).
    // Actions: Specify the actions to be performed when entering or exiting a state or during a transition (e.g., setting motor power, updating variables, displaying messages)
    public void updateArmState(){
        switch (armCurrentState) {
            case IDLE:
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (Math.abs(targetPosition - arm.getCurrentPosition()) > armEx.getTargetPositionTolerance()) {
                    armCurrentState = ArmState.MOVING_TO_TARGET;
                    stateTimer.reset();
                }
                break;
            case MOVING_TO_TARGET:
                arm.setTargetPosition(targetPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                int distance = targetPosition - arm.getCurrentPosition();
                double gravityCompensation = 200 * Math.cos(calculateAngleFromEncoderValue(arm.getCurrentPosition())); // Adjust gravity compensation as needed
                double velocity = 1000 + gravityCompensation;
                armEx.setVelocity(velocity);
                if (stateTimer.seconds() > ARM_POSITION_TIMEOUT) {
                    armCurrentState = ArmState.TIMEOUT;
                    stateTimer.reset();
                }
                else if (armEx.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT_AMPS){
                    armCurrentState = ArmState.STALLED;
                    stateTimer.reset();
                }
                else if (!arm.isBusy()) {
                    if (holdArm) armCurrentState = ArmState.HOLDING_POSITION;
                    else armCurrentState = ArmState.IDLE;
                }
                break;
            case HOLDING_POSITION:
                if (!holdArm) {armCurrentState = ArmState.IDLE;}
                else if (Math.abs(targetPosition - arm.getCurrentPosition()) > armEx.getTargetPositionTolerance()) {
                    armCurrentState = ArmState.MOVING_TO_TARGET;
                }
                break;
            case STALLED:
                targetPosition = arm.getCurrentPosition();
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (stateTimer.seconds() > 1 && armEx.getCurrent(CurrentUnit.AMPS) < MAX_CURRENT_AMPS) armCurrentState = ArmState.IDLE;
                break;
            case TIMEOUT:
                targetPosition = arm.getCurrentPosition();
                if(stateTimer.seconds() > 1) armCurrentState = ArmState.IDLE;
            case ERROR:
                telemetry.addData("ERROR:", " arm state unable to be read");
                telemetry.update();
                break;
        }
    }

    // Public method that Opmodes can call to set the target position for the state machine
    public void setArmAngle(double targetAngle){
            this.targetPosition = (int) calculateArmEncoderValue(targetAngle - ARM_STARTING_ANGLE_OFFSET);
    }

    // Public method that Opmodes can call to get what state the arm is in
    public ArmState getArmState(){
        return armCurrentState;
    }

    // Public method that Opmodes can call if needed to reset the arm state
    public void resetArmState(){
        armCurrentState = ArmState.IDLE;
    }

    public double getArmCurrentAmps(){
        return armEx.getCurrent(CurrentUnit.AMPS);
    }

    public double getArmTargetAngle(){
        return calculateAngleFromEncoderValue(targetPosition);
    }

    public double getArmTargetPosition(){
        return targetPosition;
    }
}