package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DecodeDriveMode extends LinearOpMode {
    double launcher_power = 1.0;

    private DcMotor intakeMotor;
    private DcMotor launcher;

    @Override
    public void runOpMode() {

        // Motor config
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        launcher = hardwareMap.dcMotor.get("launcherMotor");

        // Motor directions
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);

        final int CYCLE_MS = 50;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad2.y) {
                launch();
            } else {
                launcher.setPower(0.0);
            }

            double speedMultiplier = 1.0;
            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier *= 0.25;
            }

            if (gamepad1.left_bumper) {
                frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else {
                frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }

            // Drive calculations
            double y = -gamepad1.left_stick_y * speedMultiplier;
            double x = gamepad1.left_stick_x * 1.1 * speedMultiplier;
            double rx = -gamepad1.right_stick_x * speedMultiplier;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double fL_Motor = (y + x + rx) / denominator;
            double bL_Motor = (y - x + rx) / denominator;
            double fR_Motor = (y - x - rx) / denominator;
            double bR_Motor = (y + x - rx) / denominator;

            frontLeft.setPower(fL_Motor);
            backLeft.setPower(bL_Motor);
            frontRight.setPower(fR_Motor);
            backRight.setPower(bR_Motor);

            frontLeft.setVelocity(fL_Motor * 3000.0);
            backLeft.setVelocity(bL_Motor * 3000.0);
            frontRight.setVelocity(fR_Motor * 3000.0);
            backRight.setVelocity(bR_Motor * 3000.0);

            // Intake motor control
            if (gamepad2.right_stick_y != 0.0) {
                intakeMotor.setPower(gamepad2.right_stick_y);
            } else {
                intakeMotor.setPower(0.0);
            }

            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }

    private void launch() {
        if (gamepad2.dpad_up) {
            launcher_power = 0.9;
        } else if (gamepad2.dpad_left) {
            launcher_power = 0.75;
        } else if (gamepad2.dpad_right) {
            launcher_power = 0.6;
        } else if (gamepad2.dpad_down) {
            launcher_power = 0.4;
        }
        launcher.setPower(launcher_power);
    }
}
