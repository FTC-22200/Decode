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

        int Start = wristMotor.getCurrentPosition();
        int delta = 300;
        int End = Start + delta;
        Start = Start + 100;
        boolean rampUp = true;
        boolean wristIsUp = true;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double speedMultiplier = gamepad1.left_trigger > 0.5 ? 0.25 : 1.0;

            double y = -gamepad1.left_stick_y * speedMultiplier;
            double x = gamepad1.left_stick_x * 1.1 * speedMultiplier;
            double rx = -gamepad1.right_stick_x * speedMultiplier;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            frontLeft.setPower((y + x + rx) / denominator);
            backLeft.setPower((y - x + rx) / denominator);
            frontRight.setPower((y - x - rx) / denominator);
            backRight.setPower((y + x - rx) / denominator);

            if (gamepad2.right_bumper) {
                linearMotor.setPower(-0.4);
            } else if (gamepad2.right_trigger > 0) {
                linearMotor.setPower(Math.abs(gamepad2.right_trigger));
            } else {
                linearMotor.setPower(0.0);
            }

            if (gamepad2.y) {
                boxServo.setPosition(rampUp ? 0.7 : 0.2);
                rampUp = !rampUp;
            }
            while (gamepad2.y) idle();

            if (gamepad2.x) boxServo.setPosition(0.87);

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
            if (red > green && red > blue) {
                detectedColor = "RED";
            } else if (blue > red && blue > green) {
                detectedColor = "BLUE";
            } else if (green > blue && red > blue && green > 650) {
                detectedColor = "YELLOW";
            }

            String detectedColor2 = "UNKNOWN";
            if (red2 > green2 && red2 > blue2) {
                detectedColor2 = "RED";
            } else if (blue2 > red2 && blue2 > green2) {
                detectedColor2 = "BLUE";
            } else if (green2 > blue2 && red2 > blue2 && green2 > 650) {
                detectedColor2 = "YELLOW";
            }


            if (gamepad2.left_stick_y > 0.0) {
                if (detectedColor.equals("RED") || detectedColor.equals("UNKNOWN")) {
                    leftWheelServo.setPosition(0.35);
                    rightWheelServo.setPosition(0.65);
                }
                if ((detectedColor2.equals("YELLOW") || detectedColor2.equals("BLUE")) && !detectedColor.equals("YELLOW") && !detectedColor.equals("RED")) {
                    leftWheelServo.setPosition(0.499999999);
                    rightWheelServo.setPosition(0.500000001);
                }
                if (detectedColor.equals("YELLOW") || detectedColor.equals("BLUE")) {
                    for(int ant = 0; ant<=1;ant++) {
                        leftWheelServo.setPosition(0.5);
                        rightWheelServo.setPosition(0.5);
                        break;
                    }
                }


            } else if (gamepad2.left_stick_y < 0.0) {
                leftWheelServo.setPosition(Math.abs(gamepad2.left_stick_y)/2 + 0.5);
                rightWheelServo.setPosition(0.5-Math.abs(gamepad2.left_stick_y) / 2);
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
