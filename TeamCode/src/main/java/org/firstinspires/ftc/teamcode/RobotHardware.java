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

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This file works in conjunction with the External Hardware Class sample called: org.firstinspires.ftc.teamcode.CompetitionTeleop.java
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
 * Also add a new OpMode, select the sample org.firstinspires.ftc.teamcode.CompetitionTeleop.java, and select TeleOp.
 *
 */

public class RobotHardware {



    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive   = null;
    private DcMotor leftRearDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor rightRearDrive  = null;
    private DcMotor arm = null;
    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;
    private DcMotorEx armEx = null;
    private DcMotorEx liftLeftEx = null;
    private DcMotorEx liftRightEx = null;
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
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REVOL * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     MAX_CURRENT_AMPS        = 6;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    /** Mr. Morris: TO DO: test and update servo speeds. */
    public static final double GRIPPER_INCREMENT = 0.06, GRIPPER_MAX = 1, GRIPPER_MIN = 0 ;  // sets rate to move gripper servo and max and min travel. If you use SRS servo programmer to set limits, this will be 1 and 0. If you need to limit travel in the software, this is where to do it.
    public static final double WRIST_INCREMENT = 0.02 ; // sets rate to move wrist servo
    public static final double WRIST_MAX_ANGLE  = 300 ; // Adjust this angle if SRS servo programmer has limited servo travel to less than 300
    public static final int ARM_INCREMENT_DEGREES = 10, ARM_ROTATE_MAX = 160, ARM_ROTATE_MIN = -20, // Straight forward defined as 0 degrees
                            ARM_ROTATE_ENCODER_RESOLUTION = 28, ARM_ROTATE_GEAR_RATIO = 60, ARM_STARTING_ANGLE_OFFSET = 160;
    private static final int ARM_STOW_POSITION = ARM_STARTING_ANGLE_OFFSET;
    private static final long ARM_POSITION_TIMEOUT = 3000;
    private static final long LIFT_POSITION_TIMEOUT = 3000;
    public static final int LIFT_ENCODER_RESOLUTION = 28, LIFT_GEAR_RATIO = 20;
    public static final double LIFT_TRAVEL_PER_ROTATION_INCHES = 120 / 25.4, LIFT_EXTEND_INCREMENT_INCHES = 0.50, LIFT_RETRACT_INCREMENT_INCHES = -0.50;
    private static final double LIFT_FULLY_RETRACTED = 0, LOW_TO_HIGH_RUNG = 16, LOW_RUNG = 20, LOW_BASKET = 25.75, HIGH_BASKET = 43, LIFT_3_STAGE_EXTENDED = 28.8;
    public final double[] liftPositions = {LIFT_FULLY_RETRACTED, LOW_RUNG, LOW_BASKET, HIGH_BASKET, LIFT_3_STAGE_EXTENDED};
    public int liftPositionIndex = 0;
    public static boolean opModeActive = false;
    private static int armTargetPosition = 0;
    public int liftTargetPosition = 0;
    private int leftDistanceToTarget =0;
    private int rightDistanceToTarget =0;
    ElapsedTime armStateTimer = new ElapsedTime();
    ElapsedTime liftStateTimer = new ElapsedTime();

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(OpMode opmode, Telemetry telemetry) {
        this.telemetry = telemetry;
        myOpMode = opmode;
    }

    // Create state machines to track what state the arm and lift motors are in. A state machine is a computational model that represents a system
    // with a finite number of states and transitions between those states. It's a powerful tool for managing complex logic and behavior,
    // especially in systems with dynamic or event-driven interactions. Check out the updateArmState function to see how this works.
    private enum ArmState { IDLE, MOVING_TO_TARGET, HOLDING_POSITION, STALLED, TIMEOUT, STOW, ERROR}
    private ArmState armCurrentState = ArmState.IDLE;
    public boolean holdArm = false; // consider triggering the hold behavior in the opMode with a button press. TRUE: maintain arm power after RUN_TO_POSITION is complete, FALSE: stop arm power
    private enum LiftState { IDLE, MOVING_UP, MOVING_DOWN, HOLDING_POSITION, PRE_HANG_ROUTINE, HANG_ROUTINE, STALLED, TIMEOUT, ERROR}
    private LiftState liftCurrentState = LiftState.IDLE;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive  = myOpMode.hardwareMap.get(DcMotor.class, "right_rear_drive");
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        liftLeft = myOpMode.hardwareMap.get(DcMotor.class, "lift_left");
        liftRight = myOpMode.hardwareMap.get(DcMotor.class, "lift_right");

        // Take advantage of the extended features of the DcMotorEx class, such as reading current draw and setting velocity, if the motor supports it
        if (arm instanceof DcMotorEx) {
            armEx = (DcMotorEx) arm;
        } else telemetry.addData("WARNING: ", "Arm is not a DcMotorEx");
        if (liftLeft instanceof DcMotorEx) {
            liftLeftEx = (DcMotorEx) liftLeft;
        } else telemetry.addData("WARNING: ", "Left lift is not a DcMotorEx");
        if (liftRight instanceof DcMotorEx) {
            liftRightEx = (DcMotorEx) liftRight;
        } else telemetry.addData("WARNING: ", "Right lift is not a DcMotorEx");

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
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Since there are encoders connected, RUN_USING_ENCODER mode is enabled for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        gripper = myOpMode.hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(GRIPPER_MIN);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    // Drive Code
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
    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     * @param speed    speed of the move (-1.0 to 1.0) +ve is forward
     * @param forwardInches how many inches forward (or negative for backwards) to travel
     * @param rightInches how many inches right (or negative for left) to travel
     * @param rotateAngle how many degrees to rotate clockwise (or negative for counter-clockwise)
     */
    public void encoderDrive(double speed, double forwardInches, double rightInches, double rotateAngle){

    }
    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
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

    // Lift Code
    public void updateLiftState(){
        switch (liftCurrentState) {
            case IDLE:
                liftLeft.setPower(0);
                liftRight.setPower(0);
                liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // If either motor is far enough from the target distance then go into moving up state or moving down state as needed.
                if (Math.abs(leftDistanceToTarget) > liftLeftEx.getTargetPositionTolerance() || Math.abs(rightDistanceToTarget) > liftRightEx.getTargetPositionTolerance()) {
                    if ((leftDistanceToTarget + rightDistanceToTarget)/2 >= 0) liftCurrentState = LiftState.MOVING_UP;
                    else if ((leftDistanceToTarget + rightDistanceToTarget)/2 < 0) liftCurrentState = LiftState.MOVING_DOWN;
                    liftStateTimer.reset();
                }
                break;
            // The moving up state is the same as the moving down state, except the moving down state resets its lower limit when it stalls
            case MOVING_UP:
                liftLeft.setTargetPosition(liftTargetPosition);
                liftRight.setTargetPosition(liftTargetPosition);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (liftStateTimer.seconds() > LIFT_POSITION_TIMEOUT) {
                    liftCurrentState = LiftState.TIMEOUT;
                    liftStateTimer.reset();
                }
                else if (liftLeftEx.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT_AMPS){
                    liftCurrentState = LiftState.STALLED;
                    liftStateTimer.reset();
                }
                else if (!liftLeft.isBusy()) {
                    liftCurrentState = LiftState.HOLDING_POSITION;
                }
                break;
            // The moving down state is different from the hang routine because it is not expecting much resistance on the motors until it hits
            // the max retraction lower limit, so it uses the stall condition to reset the encoder
            case MOVING_DOWN:
                liftLeft.setTargetPosition(liftTargetPosition);
                liftRight.setTargetPosition(liftTargetPosition);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (liftStateTimer.seconds() > LIFT_POSITION_TIMEOUT) {
                    liftCurrentState = LiftState.TIMEOUT;
                    liftStateTimer.reset();
                }
                else if (liftLeftEx.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT_AMPS){
                    liftCurrentState = LiftState.STALLED;
                    liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftStateTimer.reset();
                }
                else if (!liftLeft.isBusy()) {
                    liftCurrentState = LiftState.IDLE;
                }
                break;
            case HOLDING_POSITION: // Not currently used
                break;
            // need to set up hang routine - this won't look for a stall condition, or at least it will have a higher max current allowed as it's expected that the motors will have to work hard to lift the whole robot.
            case PRE_HANG_ROUTINE:
                liftLeft.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                liftRight.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                armCurrentState = ArmState.STOW;
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case HANG_ROUTINE:
                liftLeft.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                liftRight.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(liftLeft.isBusy() || liftRight.isBusy())
                sleep(1000);
                liftLeft.setTargetPosition(calculateEncoderPositionFromLiftInches(LOW_TO_HIGH_RUNG));
                liftRight.setTargetPosition(calculateEncoderPositionFromLiftInches(LOW_TO_HIGH_RUNG));
                sleep(1000);
                liftLeft.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                liftRight.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                sleep(6000);
                break;
            case STALLED:
                liftTargetPosition = liftLeft.getCurrentPosition(); // pick the left lift, either will do - hopefully they're pretty much the same anyway but the other states will take care of that if they are.
                liftLeft.setPower(0);
                liftRight.setPower(0);
                liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (liftStateTimer.seconds() > 1 && liftLeftEx.getCurrent(CurrentUnit.AMPS) < MAX_CURRENT_AMPS
                        && liftRightEx.getCurrent(CurrentUnit.AMPS) < MAX_CURRENT_AMPS) liftCurrentState = LiftState.IDLE;
                break;
            case TIMEOUT:
                liftTargetPosition = liftLeft.getCurrentPosition(); // pick the left lift, either will do - hopefully they're pretty much the same anyway but the other states will take care of that if they are.
                if(liftStateTimer.seconds() > 1) {
                    liftCurrentState = LiftState.IDLE;
                    liftStateTimer.reset();
                }
                break;
            // There isn't currently any way the error state triggers. We may be able to get rid of this, but it would be good to get more error handling in the code, so I'm leaving it for now as a reminder
            case ERROR:
                telemetry.addData("ERROR", " lift state unable to be read");
                telemetry.update();
                break;
        }
    }
    // Public method that Opmodes can call to set the target position for the lift state machine
    public void setLiftPosition(double targetPosition){
        this.liftTargetPosition = targetPosition;
        this.leftDistanceToTarget = targetPosition - liftLeft.getCurrentPosition();
        this.rightDistanceToTarget = targetPosition - liftRight.getCurrentPosition();
    }
    public void setLiftPositionInches(double targetPositionInches) {
        setLiftPosition(calculateEncoderPositionFromLiftInches(targetPositionInches));
    }
    public void liftIncrementInches(){
        if (getLiftExtensionInches() < LIFT_3_STAGE_EXTENDED) {
            setLiftPositionInches(getLiftExtensionInches() + LIFT_EXTEND_INCREMENT_INCHES);
        }
    }
    public void liftDecrementInches(){
        if (getLiftExtensionInches() > LIFT_FULLY_RETRACTED) {
            setLiftPositionInches(getLiftExtensionInches() - LIFT_RETRACT_INCREMENT_INCHES);
        }
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
        return calculateLiftInchesFromEncoderPosition(liftLeft.getCurrentPosition());
    }
    // Public method that Opmodes can call to get what state the lift is in
    public LiftState getLiftState(){
        return liftCurrentState;
    }
    // Public method that Opmodes can call to execute the pre-hang routine
    public void preHangRoutine(){
        liftCurrentState = LiftState.PRE_HANG_ROUTINE;
    }
    // Public method that Opmodes can call to execute the hang routine
    public void hangRoutine(){
        liftCurrentState = LiftState.HANG_ROUTINE;
    }
    public double getLiftLeftCurrentAmps(){
        return liftLeftEx.getCurrent(CurrentUnit.AMPS);
    }
    public double getLiftRightCurrentAmps(){
        return liftRightEx.getCurrent(CurrentUnit.AMPS);
    }
    public void resetLiftState(){
        liftCurrentState = LiftState.IDLE;
    }

    // Gripper Code
    /**
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

    // Wrist Code
    /**
     * Send the wrist to a certain position
     * @param position is a number from 0 to 1
     */
    public void setWristPosition(double position){
       wrist.setPosition(position);
    }
    public void setWristAngle(double angle){
        setWristPosition(RangeUtils.scale(angle, 0, WRIST_MAX_ANGLE, 0, 1));
    }
    public double getWristPosition(){
        return wrist.getPosition();
    }
    public void wristIncrement() {
        setWristPosition(Math.min(1, wrist.getPosition() + WRIST_INCREMENT));
    }
    public void wristDecrement() {
        if (wrist.getPosition() > 0) {
            setWristPosition(Math.max(0, wrist.getPosition() - WRIST_INCREMENT));
        }
    }
    public double getWristAngle(){
        return RangeUtils.scale(wrist.getPosition(), 0,1, 0, WRIST_MAX_ANGLE);
    }

    // Arm Code
    // The following function creates a state machine. A state machine is a computational model that represents a system with a finite number of states and transitions between
    // those states. It's a powerful tool for managing complex logic and behavior, especially in systems with dynamic or event-driven interactions.
    // Key Components:
    // States: Represent the different possible conditions or modes of the system (e.g., "idle," "moving," "holding," "error").
    // Transitions: Define the rules for moving from one state to another based on events or conditions (e.g., sensor readings, user input, timeouts).
    // Actions: Specify the actions to be performed when entering or exiting a state or during a transition (e.g., setting motor power, updating variables, displaying messages)
    public void updateArmState(){
        switch (armCurrentState) {
            // In the idle state it shuts off power to the motor as long as it stays close to its target position,
            // this saves battery but is more prone to drooping and wobble from engaging & disengaging the motor
            case IDLE:
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (Math.abs(armTargetPosition - arm.getCurrentPosition()) > armEx.getTargetPositionTolerance()) {
                    armCurrentState = ArmState.MOVING_TO_TARGET;
                    armStateTimer.reset();
                }
                break;
            case MOVING_TO_TARGET:
                arm.setTargetPosition(armTargetPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                int distance = armTargetPosition - arm.getCurrentPosition(); // This was used previously to compensate velocity to speed up small moves. velocity = 500 + 500* (1-distance
                double gravityCompensation = 300 * Math.cos(Math.toRadians(calculateAngleFromEncoderValue(arm.getCurrentPosition()))); // Adjust gravity compensation as needed
                double velocity = 1000;
                if (distance >= 0) {velocity += gravityCompensation;}
                else {velocity -= gravityCompensation;}
                armEx.setVelocity(velocity);
                if (armStateTimer.seconds() > ARM_POSITION_TIMEOUT) {
                    armCurrentState = ArmState.TIMEOUT;
                    armStateTimer.reset();
                }
                else if (armEx.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT_AMPS){
                    armCurrentState = ArmState.STALLED;
                    armStateTimer.reset();
                }
                else if (!arm.isBusy()) {
                    if (holdArm) armCurrentState = ArmState.HOLDING_POSITION;
                    else armCurrentState = ArmState.IDLE;
                }
                break;
            // The holding position state only happens if holdArm is toggled true (currently no button assigned in MorrisPOVDrive)
            // it will keep power on to the motor the whole time, which helps maintain position more rigidly but draws more power.
            case HOLDING_POSITION:
                if (!holdArm) {armCurrentState = ArmState.IDLE;}
                else if (Math.abs(armTargetPosition - arm.getCurrentPosition()) > armEx.getTargetPositionTolerance()) {
                    armCurrentState = ArmState.MOVING_TO_TARGET;
                }
                break;
            case STALLED:
                armTargetPosition = arm.getCurrentPosition();
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (armStateTimer.seconds() > 1 && armEx.getCurrent(CurrentUnit.AMPS) < MAX_CURRENT_AMPS) armCurrentState = ArmState.IDLE;
                break;
            case TIMEOUT:
                armTargetPosition = arm.getCurrentPosition();
                if(armStateTimer.seconds() > 1){
                    armCurrentState = ArmState.IDLE;
                    armStateTimer.reset();
                }
            case STOW:
                armTargetPosition = ARM_STOW_POSITION;
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            // There isn't currently any way the error state triggers. We may be able to get rid of this, but it would be good to get more error handling in the code, so I'm leaving it for now as a reminder
            case ERROR:
                telemetry.addData("ERROR:", " arm state unable to be read");
                telemetry.update();
                break;
        }
    }
    // Public method that Opmodes can call to set the target position for the arm state machine
    public void setArmAngle(double targetAngle){
            this.armTargetPosition = (int) calculateArmEncoderValue(targetAngle - ARM_STARTING_ANGLE_OFFSET);
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
            setArmAngle(getArmAngleRelativeToZero() + ARM_INCREMENT_DEGREES);
        }
    }
    public void armAngleDecrement(){
        if (getArmAngleRelativeToZero() > ARM_ROTATE_MIN){
            setArmAngle(getArmAngleRelativeToZero() - ARM_INCREMENT_DEGREES);
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
    public double getArmEncoderCounts(){
        return arm.getCurrentPosition();
    }
    public double getArmTargetAngle(){
        return calculateAngleFromEncoderValue(armTargetPosition);
    }
    public double getArmTargetPosition(){
        return armTargetPosition;
    }

    public static class RangeUtils {
        public static double scale(double value, double inputMin, double inputMax, double outputMin, double outputMax) {
            return outputMin + (value - inputMin) * (outputMax - outputMin) / (inputMax - inputMin);
        }
        public static double clip( double value, double min, double max) {
            if (value < min) return min;
            else if (value > max) return max;
            else return value;
        }
    }
}