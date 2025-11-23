package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.SerialNumber;

@TeleOp
public class DecodeDriveMode extends LinearOpMode {
    private DcMotorEx frontLeft = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backRight = null;
    private CRServo boxServo = null;
    @Override
    public void runOpMode() {

        // Motor config
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        boxServo = hardwareMap.get(CRServo.class, "boxServo");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        // Servo config

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

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

            frontLeft.setVelocity(fL_Motor*2000.0);
            backLeft.setVelocity(bL_Motor*2000.0);
            frontRight.setVelocity(fR_Motor*2000.0);
            backRight.setVelocity(bR_Motor*2000.0);
            frontLeft.setPower(fL_Motor);
            backLeft.setPower(bL_Motor);
            frontRight.setPower(fR_Motor);
            backRight.setPower(bR_Motor);

            // Linear motor control

            // Run intakeMotor
            if (gamepad2.right_stick_y < 0.0) {
                intakeMotor.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y > 0.0) {
                intakeMotor.setPower(gamepad2.right_stick_y);
            } else {
                intakeMotor.setPower(0.0);
            }

            // Box servo to push the ball into the box
            if (gamepad2.a) {
                boxServo.setPower(0.6);
            } else {
                boxServo.setPower(0.0);
            }
                telemetry.update();
                sleep(CYCLE_MS);
                idle();
            }
        }
    }


