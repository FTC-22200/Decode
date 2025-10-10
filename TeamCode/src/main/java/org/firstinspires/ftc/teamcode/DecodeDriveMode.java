package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DecodeDriveMode extends LinearOpMode {
    @Override
    public void runOpMode() {

        // Motor config
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        // Servo config

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        final int CYCLE_MS = 50;

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

                telemetry.update();
                sleep(CYCLE_MS);
                idle();
            }
        }
    }


