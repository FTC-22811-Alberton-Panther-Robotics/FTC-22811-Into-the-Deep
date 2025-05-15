package org.firstinspires.ftc.teamcode.EvansWork;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.

@Autonomous(name="EvanDrive Forward", group="Exercises")
//@Disabled
public class EvanDriveForward extends LinearOpMode
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

        // set both motors to 25% power.

        left_front_drive.setPower(0.25);
        left_rear_drive.setPower(0.25);
        right_front_drive.setPower(0.25);
        right_rear_drive.setPower(0.25);

        sleep(2000);        // wait for 2 seconds.

        // set motor power to zero to stop motors.

        left_front_drive.setPower(0.0);
        left_rear_drive.setPower(0.0);
        right_front_drive.setPower(0.0);
        right_rear_drive.setPower(0.0);
    }
}
