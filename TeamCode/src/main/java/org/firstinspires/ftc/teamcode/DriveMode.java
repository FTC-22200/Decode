package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveMode extends LinearOpMode {
    @Override
    public void runOpMode() {

        // Motor config
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor linearMotor = hardwareMap.dcMotor.get("linearMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        // Servo config
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo intakeServo = hardwareMap.get(Servo.class, "intakeServo");

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

        // Define class members
        double position2 = 1.0; // Start at maximum position
        boolean rampUp = true;
        boolean boxUp = false;
        boolean boxDown = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // For forwards/backwards movement
            double x = -gamepad1.left_stick_x * 1.1; // The 1.1 multiplier is to counteract imperfect strafing
            double rx = gamepad1.right_stick_x; // Turning left/right

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
                linearMotor.setPower(-1.0); // Reverse if right bumper pressed
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
            double intakePower = intakeMotor.getPower();
            if (gamepad2.dpad_up) {
                // Move forward by 0.01
                intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                intakePower += 0.05;
                if (intakePower > 1.0) {
                    intakePower = 1.0; // Ensure the power doesn't exceed the maximum
                }
            } else if (gamepad2.dpad_down) {
                // Move backward by 0.01
                intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                intakePower += 0.05;
                if (intakePower > 1.0) {
                    intakePower = 1.0; // Ensure the power doesn't exceed the maximum
                }
            }
            intakeMotor.setPower(intakePower);

            // Wrist servo control
            if (gamepad2.b) {
                wristServo.setPosition(0.65); // Changed from 1.8 to 1.0, assuming 1.0 is maximum
            } else if (gamepad2.a) {
                wristServo.setPosition(0.15); // Adjusted to a valid position
            }

            // Intake servo control
            if (gamepad2.left_stick_y < 0) {
                intakeServo.setPosition(1.0); // Full forward
            } else if (gamepad2.left_stick_y > 0) {
                intakeServo.setPosition(0.0); // Reverse the servo
            } else if (gamepad2.left_stick_y == 0) {
                intakeServo.setPosition(0.5); // Stop the servo
            }

            // Update telemetry
            telemetry.addData("Box Down", boxDown);
            telemetry.addData("Box Up", boxUp);
            telemetry.addData("Ramp Up", rampUp);
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }
}
