package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "ArmGravityTest", group = "Test")
public class ArmGravityTest extends LinearOpMode {
    private DcMotorEx motor;
    private double f_base = 1.16; // Initial f_base value
    private double currentAngleDegrees = 0.0;
    private double targetAngle = 0;
    double p_dynamic = 10; // Default is 10

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motor and other hardware
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

//        // Initial lift to a test position (e.g., horizontal)
//        // You can use a temporary power setting here
//        motor.setPower(0.3); // Adjust this power as needed
//        sleep(2000); // Let it move for a bit
//        motor.setPower(0); // Stop the initial movement

        // Set the initial target position to the current position
        targetAngle = getArmAngle();

        while (opModeIsActive()) {
            // Get the current arm angle
            currentAngleDegrees = getArmAngle(); // Replace with your method to get the angle

            // Calculate the dynamic feedforward value
            double f_dynamic = calculateDynamicFeedforward(f_base, currentAngleDegrees);

            // Set the position PIDF coefficients - default is (10, 0.049988, 0, 0) - this one uses the velocity coefficients as well so the only one I can set is the p term
//            motor.setPositionPIDFCoefficients(p_dynamic);
//            // Set the velocity PIDF coefficients - default is (10, 3, 0, 0)
//            motor.setVelocityPIDFCoefficients(p_dynamic,3,0,f_dynamic);


            // Set the target position to the current position
            motor.setTargetPosition((int)angleToEncoder(targetAngle)); // Cast to int
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(.5); // Adjust this power as needed

            // Handle button presses to adjust f_base
            handleButtonPresses();

            // Telemetry
            telemetry.addData("Position PIDF", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("Velocity PIDF", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("f_base", f_base);
            telemetry.addData("f_dynamic", f_dynamic);
            telemetry.addData("p_dynamic", p_dynamic);
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Motor Current", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current Angle", currentAngleDegrees);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Target Position", angleToEncoder(targetAngle));
            telemetry.update();
        }
    }

    private double calculateDynamicFeedforward(double f_base, double currentAngleDegrees) {
        double angleRadians = Math.toRadians(currentAngleDegrees);
        double cosAngle = Math.cos(angleRadians);
        return f_base * cosAngle;
    }

    // Function to calculate arm angle from encoder position
    private double getArmAngle() {
        return encoderToAngle(motor.getCurrentPosition());
    }

    private double encoderToAngle(int encoderTicks) {
        double ticksPerRevolution = 28 * 60;
        double degreesPerTick = 360.0 / ticksPerRevolution;
        return encoderTicks * degreesPerTick;
    }
    private double angleToEncoder(double angleDegrees) {
        double ticksPerRevolution = 28 * 60;
        double degreesPerTick = 360.0 / ticksPerRevolution;
        return angleDegrees / degreesPerTick;
    }

    private void handleButtonPresses() {
        if (gamepad1.dpad_up) {
            f_base += 0.2; // Increment f_base by a small amount
            while (gamepad1.dpad_up) {
                // Wait for button release
            }
        }
        if (gamepad1.dpad_down) {
            f_base -= 0.2; // Decrement f_base by a small amount
            while (gamepad1.dpad_down) {
                // Wait for button release
            }
        }
        if (gamepad1.dpad_left){
            targetAngle -= 5;
            while (gamepad1.dpad_left) {
                // Wait for button release
            }
        }
        if (gamepad1.dpad_right) {
            targetAngle += 5;
            while (gamepad1.dpad_right) {
                // Wait for button release
            }
        }
        if (gamepad1.left_bumper) {
            p_dynamic -= 0.2; // Increment i_dynamic by a small amount
            while (gamepad1.left_bumper) {
                // Wait for button release
            }
        }
        if (gamepad1.right_bumper) {
            p_dynamic += 0.2; // Decrement i_dynamic by a small amount
            while (gamepad1.right_bumper) {
                // Wait for button release
            }
        }
    }
}