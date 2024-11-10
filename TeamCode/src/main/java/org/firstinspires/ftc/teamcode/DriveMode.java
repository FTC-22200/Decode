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
        // Servo config
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        linearMotor.setDirection(DcMotor.Direction.FORWARD);


        final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
        final int    CYCLE_MS    =   50;     // period of each cycle
        final double MAX_POS     =  1.0;     // Maximum rotational position
        final double MIN_POS     =  0.0;     // Minimum rotational position

        // Define class members
        double  position = MIN_POS; // Start at halfway position
        boolean rampUp = true;
        boolean boxUp = false;
        boolean boxDown = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // For forwards/backwards movement
            double x = -gamepad1.left_stick_x * 1.1; // The 1.1 multiplier is to counteract imperfect strafing
            double rx = gamepad1.right_stick_x; // Turning left/right


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); /* makes sure motor values don't go outside of [-1,1] */
            double fL_Motor = (y+x+rx)/denominator; // fL = FrontLeft
            double bL_Motor = (y-x+rx)/denominator; // bL = BackLeft
            double fR_Motor = (y-x-rx)/denominator; // fR = FrontRight
            double bR_Motor = (y+x-rx)/denominator; // bR = backRight

            // Make sure x is pressed and y is not pressed
            if (gamepad2.x & !gamepad2.y & !boxDown & !boxUp) {
                boxUp = true;
            } else if (gamepad2.y & !boxDown & !boxUp) {
                boxDown = true;
            }
            if (boxUp & boxDown) {
                boxDown = false;
                boxUp = false;
            }
            if (boxUp) {
                while (rampUp) {
                    position += INCREMENT;
                    if (position >= MAX_POS) {
                        rampUp = false;
                        boxUp = false;
                        position = MAX_POS;
                    }
                }
            }
            if (boxDown) {
                while (!rampUp) {
                    position -= INCREMENT;
                    if (position <= MIN_POS) {
                        rampUp = true;
                        boxDown = false;
                        position = MIN_POS;
                    }
                }
            }

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
                linearMotor.setPower(0); // Stop linear motor if no input
            }
            boxServo.setPosition(position);
            sleep(CYCLE_MS);
            idle();
        }
    }
}


