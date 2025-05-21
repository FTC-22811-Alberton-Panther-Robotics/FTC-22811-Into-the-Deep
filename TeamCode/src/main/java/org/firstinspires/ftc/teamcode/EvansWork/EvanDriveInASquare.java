package org.firstinspires.ftc.teamcode.EvansWork;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Evan Drive Square", group="Exercises")
//@Disabled
public class EvanDriveInASquare extends LinearOpMode
{
    DcMotor left_front_drive;
    DcMotor left_rear_drive;
    DcMotor right_front_drive;
    DcMotor right_rear_drive;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        left_front_drive = hardwareMap.dcMotor.get("left_front_drive");
        left_rear_drive = hardwareMap.dcMotor.get("left_rear_drive");
        right_front_drive = hardwareMap.dcMotor.get("right_front_drive");
        right_rear_drive = hardwareMap.dcMotor.get("right_rear_drive");

        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_rear_drive.setDirection(DcMotor.Direction.REVERSE);

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

            left_front_drive.setPower(0.25);
            left_rear_drive.setPower(0.25);
            right_front_drive.setPower(0.25);
            right_rear_drive.setPower(0.25);
            sleep(500); // drive straight for 1 second.

            left_front_drive.setPower(0.0);
            left_rear_drive.setPower(0.0);
            right_front_drive.setPower(0.0);
            right_rear_drive.setPower(0.0);
            sleep(500);  // wait half second for bot to stop moving.

            // now set motors, one forward one reverse. Should cause the bot to rotate.


            left_front_drive.setPower(0.25);
            left_rear_drive.setPower(0.25);
            right_front_drive.setPower(-0.25);
            right_rear_drive.setPower(-0.25);

            sleep(1500); // adjust this delay to get the bot to rotate 90 degrees.

            left_front_drive.setPower(0.0);
            left_rear_drive.setPower(0.0);
            right_front_drive.setPower(0.0);
            right_rear_drive.setPower(0.0);

            sleep(500); // wait for bot to stop moving.
        }

        // make sure the motors are off.
        left_front_drive.setPower(0.0);
        left_rear_drive.setPower(0.0);
        right_front_drive.setPower(0.0);
        right_rear_drive.setPower(0.0);
    }
}