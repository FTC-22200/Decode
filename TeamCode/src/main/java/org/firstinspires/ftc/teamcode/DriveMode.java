package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriveMode")
public class DriveMode extends LinearOpMode {
    @Override
    public void runOpMode() {

        // Motor config
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Servo config
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo leftWheelServo = hardwareMap.get(Servo.class, "leftWheelServo");
        Servo rightWheelServo = hardwareMap.get(Servo.class, "rightWheelServo");

        // Set default positions
        leftWheelServo.setPosition(0.5);
        rightWheelServo.setPosition(0.5);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        linearMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        final double INCREMENT2 = 0.1;     // amount to slew servo each CYCLE_MS cycle
        final int CYCLE_MS = 50;           // period of each cycle
        final double MAX_POS2 = 0.4;       // Maximum rotational position
        final double MIN_POS2 = 1.0;       // Minimum rotational position
        double maxSafeTemperature = 75.0; // Define a maximum safe temperature

        // Define class members
        double position2 = 1.0; // Start at maximum position
        boolean rampUp = true;
        boolean boxUp = false;
        boolean boxDown = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double speedMultiplier = 1.0;

            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier = 0.25; // Reduce speed by a quarter when you hold left trigger
            }
            double y = -gamepad1.left_stick_y * speedMultiplier; // For forwards/backwards movement
            double x = gamepad1.left_stick_x * 1.1 * speedMultiplier; // The 1.1 multiplier is to counteract imperfect strafing
            double rx = -gamepad1.right_stick_x * speedMultiplier; // Turning left/right
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); // Ensures motor values stay within [-1, 1]
            double fL_Motor = (y + x + rx) / denominator; // fL = FrontLeft
            double bL_Motor = (y - x + rx) / denominator; // bL = BackLeft
            double fR_Motor = (y - x - rx) / denominator; // fR = FrontRight
            double bR_Motor = (y + x - rx) / denominator; // bR = BackRight

            frontLeft.setPower(fL_Motor);
            backLeft.setPower(bL_Motor);
            frontRight.setPower(fR_Motor);
            backRight.setPower(bR_Motor);

            // Linear motor control
            if (gamepad2.right_bumper) {
                linearMotor.setPower(-0.4); // Reverse if right bumper pressed
            } else if (gamepad2.right_trigger > 0) {
                linearMotor.setPower(Math.abs(gamepad2.right_trigger)); // Forward with right trigger
            } else {
                linearMotor.setPower(0.0); // Stop linear motor if no input
            }

            // Box servo control
            if (gamepad2.x && !gamepad2.y && !boxDown && !boxUp && !rampUp) {
                boxDown = true;
            }
            if (gamepad2.y && !gamepad2.x && !boxDown && !boxUp && rampUp) {
                boxUp = true;
            }
            if (boxUp && boxDown) {
                boxDown = false;
                boxUp = false;
            }
            if (boxUp) {
                position2 -= INCREMENT2;
                if (position2 <= MAX_POS2) {
                    rampUp = false;
                    boxUp = false;
                    position2 = MAX_POS2;
                }
            }

            if (boxDown) {
                position2 += INCREMENT2;
                if (position2 >= MIN_POS2) {
                    rampUp = true;
                    boxDown = false;
                    position2 = MIN_POS2;
                }
            }
            boxServo.setPosition(position2);

            // Intake motor control
            if (gamepad2.dpad_up) {
                intakeMotor.setPower(1);
            } else if (gamepad2.dpad_down) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            // Wrist servo control
            if (gamepad2.b) {
                wristServo.setPosition(0.65); // Changed from 1.8 to 1.0, assuming 1.0 is maximum
            } else if (gamepad2.a) {
                wristServo.setPosition(0.15); // Adjusted to a valid position
            } else if (gamepad2.dpad_right) {
                wristServo.setPosition(0);
            }

            // Intake servo control
            if (gamepad2.left_stick_y < 0) {
                leftWheelServo.setPosition(1.0); // Full forward
                rightWheelServo.setPosition(0.0);
            } else if (gamepad2.left_stick_y > 0) {
                leftWheelServo.setPosition(0.0); // Reverse the servo
                rightWheelServo.setPosition(1.0);
            } else if (gamepad2.left_stick_y == 0) {
                leftWheelServo.setPosition(0.5); // Stop the servo
                rightWheelServo.setPosition(0.5);
            }

            // Optional: Add telemetry to display servo positions
            telemetry.addData("Left Wheel Servo Position", leftWheelServo.getPosition());
            telemetry.addData("Right Wheel Servo Position", rightWheelServo.getPosition());
            telemetry.addData("Box Down", boxDown);
            telemetry.addData("Box Up", boxUp);
            telemetry.addData("Ramp Up", rampUp);

            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }
}
