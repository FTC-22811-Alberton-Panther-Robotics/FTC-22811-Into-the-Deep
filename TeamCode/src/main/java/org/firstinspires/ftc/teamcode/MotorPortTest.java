package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;

@TeleOp(name = "MotorPortTest", group = "Test")
public class MotorPortTest extends LinearOpMode {
    private int motorPortNumber = 0; // Change this to the port number of your motor
    private int selectionIndex = 0;
    private DcMotorControllerEx motorController;
    String[] hubOptions = {"Control Hub", "Expansion Hub 2"};
    String selectedHub = null;
    private boolean dpadRightPressed = false;
    private boolean dpadLeftPressed = false;
    private double power = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            // Access the motor by its port number
            boolean exceptionCaught = false;
            selectionIndex = selectionIndex % 8;
            motorPortNumber = selectionIndex % 4;
            if (selectionIndex / 4 == 0) selectedHub = hubOptions[0];
            else selectedHub = hubOptions[1];
            try {
                motorController = hardwareMap.get(DcMotorControllerEx.class, selectedHub);
                motorController.setMotorMode(motorPortNumber, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (IllegalArgumentException e) {
                telemetry.addData("Error", "Motor not found on" + selectedHub + " port " + motorPortNumber);
                exceptionCaught = true;
            }

            if (!exceptionCaught) {
                // Control the motor with gamepad input
                power = gamepad1.left_stick_y;
                motorController.setMotorPower(motorPortNumber, power);
            }

            if (gamepad1.dpad_right && !dpadRightPressed) {
                selectionIndex++;
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                selectionIndex--;
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }
            // Telemetry
            telemetry.addData("Selected Hub", selectedHub);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Selection Index", selectionIndex);
            telemetry.addData("Motor Port", motorPortNumber);
            telemetry.update();
        }
    }
}