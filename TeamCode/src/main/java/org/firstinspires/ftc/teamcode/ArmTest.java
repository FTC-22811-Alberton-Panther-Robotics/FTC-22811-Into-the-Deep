package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ArmTest", group="Test Code")
public class ArmTest extends OpMode {

    private DcMotor arm_rotate;

    @Override
    public void init() {
        arm_rotate = hardwareMap.get(DcMotor.class, "arm_rotate");
        // Set the motor's direction (forward or reverse)
        arm_rotate.setDirection(DcMotor.Direction.FORWARD);
        // Set the motor's zero power behavior (brake or float)
        arm_rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // Control the arm using gamepad input
        double power = gamepad1.left_stick_y;

        // Set the motor power
        arm_rotate.setPower(power);
    }
}