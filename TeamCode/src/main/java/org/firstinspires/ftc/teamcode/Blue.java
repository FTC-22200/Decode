package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Blue extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        ColorSensor colorSensor;
        ColorSensor colorSensor2;

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor linearMotor = hardwareMap.dcMotor.get("linearMotor");
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotorEx wristMotor = hardwareMap.get(DcMotorEx.class, "wristMotor");
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "sensor_color2");

        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");
        Servo leftWheelServo = hardwareMap.get(Servo.class, "leftWheelServo");
        Servo rightWheelServo = hardwareMap.get(Servo.class, "rightWheelServo");

        leftWheelServo.setPosition(0.5);
        rightWheelServo.setPosition(0.5);
        boxServo.setPosition(0.87);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        linearMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        wristMotor.setDirection(DcMotor.Direction.FORWARD);

        final double INCREMENT2 = 0.1;
        final double INCREMENT = 109.5;
        final int CYCLE_MS = 50;
        final double MAX_POS2 = 0.4;
        final double MIN_POS2 = 1.0;
        int Start = wristMotor.getCurrentPosition();
        int delta = 300;
        int End = Start + delta;
        Start = Start + 100;
        double maxSafeTemperature = 75.0;
        double forWheel = 0;

        double position1 = 1;
        double position2 = 1.0;
        boolean rampUp = true;
        boolean wristUp = false;
        boolean wristDown = false;
        boolean wristIsUp = true;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double speedMultiplier = 1.0;

            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier = 0.25;
            }

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

            if (gamepad2.right_bumper) {
                linearMotor.setPower(-0.4);
            } else if (gamepad2.right_trigger > 0) {
                linearMotor.setPower(Math.abs(gamepad2.right_trigger));
            } else {
                linearMotor.setPower(0.0);
            }

            if (gamepad2.y) {
                if (rampUp) {
                    boxServo.setPosition(0.7);
                    rampUp = false;
                } else {
                    boxServo.setPosition(0.2);
                    rampUp = true;
                }
            }
            while (gamepad2.y) {
                idle();
            }

            if (gamepad2.x) {
                boxServo.setPosition(0.87);
            }

            if (gamepad2.dpad_up) {
                intakeMotor.setPower(1);
            } else if (gamepad2.dpad_down) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad2.b) {
                wristMotor.setPower(0.5);
                wristIsUp = false;
            }
            if (gamepad2.a) {
                wristMotor.setPower(-0.5);
                wristIsUp = true;
            }
            if (gamepad2.left_bumper) {
                wristMotor.setPower(0);
            }
            if (wristIsUp && wristMotor.getCurrentPosition() < Start) {
                wristMotor.setPower(0);
            }
            if (!wristIsUp && wristMotor.getCurrentPosition() > End) {
                wristMotor.setPower(0);
            }

            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();

            int red2 = colorSensor2.red();
            int blue2 = colorSensor2.blue();
            int green2 = colorSensor2.green();

            String detectedColor = "UNKNOWN";
            if (blue > green && blue > red) {
                detectedColor = "RED";
            } else if (red > blue && red > green) {
                detectedColor = "BLUE";
            } else if (green > blue && red > blue && green > 650) {
                detectedColor = "YELLOW";
            }

            String detectedColor2 = "UNKNOWN";
            if (blue2 > green2 && blue2 > red2) {
                detectedColor2 = "RED";
            } else if (red2 > blue2 && red2 > green2) {
                detectedColor2 = "BLUE";
            } else if (green2 > blue2 && red2 > blue2 && green2 > 650) {
                detectedColor2 = "YELLOW";
            }

            if (gamepad2.left_stick_y > 0.0) {
                if ((detectedColor2.equals("YELLOW") || detectedColor2.equals("RED")) && !detectedColor.equals("YELLOW") && !detectedColor.equals("RED")) {
                    leftWheelServo.setPosition(0.45);
                    rightWheelServo.setPosition(0.55);
                } else if (detectedColor.equals("YELLOW") || detectedColor.equals("RED")) {
                    leftWheelServo.setPosition(0.5);
                    rightWheelServo.setPosition(0.5);
                } else if (detectedColor.equals("BLUE") || detectedColor.equals("UNKNOWN")) {
                    leftWheelServo.setPosition(0.0);
                    rightWheelServo.setPosition(1.0);
                }
            } else if (gamepad2.left_stick_y < 0.0) {
                leftWheelServo.setPosition(1.0);
                rightWheelServo.setPosition(0.0);
            } else {
                leftWheelServo.setPosition(0.5);
                rightWheelServo.setPosition(0.5);
            }

            telemetry.addData("wristMotor position", wristMotor.getCurrentPosition());
            telemetry.addData("Left Wheel Servo Position", leftWheelServo.getPosition());
            telemetry.addData("Right Wheel Servo Position", rightWheelServo.getPosition());
            telemetry.addData("box Servo Position", boxServo.getPosition());
            telemetry.addData("linearMotor position", linearMotor.getCurrentPosition());
            telemetry.addData("Ramp Up", rampUp);
            telemetry.addData("Red", red);
            telemetry.addData("Blue", blue);
            telemetry.addData("Green", green);
            telemetry.addData("detectedColor", detectedColor);
            telemetry.addData("Red2", red2);
            telemetry.addData("Blue2", blue2);
            telemetry.addData("Green2", green2);
            telemetry.addData("detectedColor2", detectedColor2);
            telemetry.update();
            sleep(20);
            idle();
        }
    }
}