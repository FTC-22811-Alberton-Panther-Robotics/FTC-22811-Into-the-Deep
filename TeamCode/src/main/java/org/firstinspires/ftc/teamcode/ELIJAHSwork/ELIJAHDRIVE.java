
package org.firstinspires.ftc.teamcode.ELIJAHSwork;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="ELIJAHDRIVE")

public class ELIJAHDRIVE extends LinearOpMode
{
    DcMotor left_front_drive;
    DcMotor left_rear_drive;
    DcMotor right_front_drive;
    DcMotor right_rear_drive;



    @Override
    public void runOpMode() throws InterruptedException
    {


        left_front_drive = hardwareMap.dcMotor.get("left_front_drive");
        left_rear_drive = hardwareMap.dcMotor.get("left_rear_drive");
        right_front_drive = hardwareMap.dcMotor.get("right_front_drive");
        right_rear_drive = hardwareMap.dcMotor.get("right_rear_drive");

        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_rear_drive.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();


        left_front_drive.setPower(0.25);
        left_rear_drive.setPower(0.25);
        right_front_drive.setPower(0.25);
        right_rear_drive.setPower(0.25);

        sleep(2000);

        left_front_drive.setPower(0.0);
        left_rear_drive.setPower(0.0);
        right_front_drive.setPower(0.0);
        right_rear_drive.setPower(0.0);

        sleep(500);








    }
}
