package org.firstinspires.ftc.teamcode.david;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="davidSquare", group="Exercises")
//@Disabled
public class davidSquare extends LinearOpMode
{
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftRearDrive;
    DcMotor rightRearDrive;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(500);        // wait so that above telemetry is visible.

        // each iteration of this for loop will drive one side of the square.

        for(int i = 0; i < 4; i++)
        {
            telemetry.addData("Mode", "driving side " + (i + 1));
            telemetry.update();

            leftFrontDrive.setPower(0.25);
            rightFrontDrive.setPower(0.25);
            leftRearDrive.setPower(0.25);
            rightRearDrive.setPower(0.25);

            sleep(1000); // drive straight for 1 second.

            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightRearDrive.setPower(0);

            sleep(500);  // wait half second for bot to stop moving.

            // now set motors, one forward one reverse. Should cause the bot to rotate.

            leftFrontDrive.setPower(0.25);
            rightFrontDrive.setPower(0.25);
            leftRearDrive.setPower(-0.25);
            rightRearDrive.setPower(-0.25);


            sleep(1700); // adjust this delay to get the bot to rotate 90 degrees.

            leftFrontDrive.setPower(0.25);
            rightFrontDrive.setPower(0.25);
            leftRearDrive.setPower(0.25);
            rightRearDrive.setPower(0.25);

            sleep(500); // wait for bot to stop moving.
        }

        // make sure the motors are off.

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }
}
