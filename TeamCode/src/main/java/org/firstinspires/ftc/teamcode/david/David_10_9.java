package org.firstinspires.ftc.teamcode.david;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.

@Autonomous(name="David 10.9", group="Exercises")
//@Disabled
public class David_10_9 extends LinearOpMode
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

        // set both motors to 25% power.

        leftFrontDrive.setPower(0.25);
        rightFrontDrive.setPower(0.25);
        leftRearDrive.setPower(0.25);
        rightRearDrive.setPower(0.25);

        sleep(2000);        // wait for 2 seconds.

        // set motor power to zero to stop motors.

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }
}
