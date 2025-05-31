package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp
public class Red extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        ColorSensor colorSensor;
        ColorSensor colorSensor2;
        // Motor config
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

        // Servo config
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");
        Servo leftWheelServo = hardwareMap.get(Servo.class, "leftWheelServo");
        Servo rightWheelServo = hardwareMap.get(Servo.class, "rightWheelServo");

        // Set default positions
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


        final double INCREMENT2 = 0.1; // amount to slew servo each CYCLE_MS cycle
        final double INCREMENT = 109.5;
        final int CYCLE_MS = 50;         // period of each cycle
        final double MAX_POS2 = 0.4;       // Maximum rotational position
        final double MIN_POS2 = 1.0; // Minimum rotational position
        int Start = wristMotor.getCurrentPosition();
        int delta = 300;
        int End = Start + delta;
        Start = Start + 100;
        double maxSafeTemperature = 75.0; // Define a maximum safe temperature
        // double forWheel = 0;

        // Define class members
        double position1 = 1;
        double position2 = 1.0; // Start at maximum position
        boolean rampUp = true;
        boolean wristUp = false;
        boolean wristDown = false;
        boolean wristIsUp = true;

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

            // Intake motor control
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
                if ((detectedColor2.equals("YELLOW") || detectedColor2.equals("RED")) && !detectedColor.equals("YELLOW") && !detectedColor.equals("RED")) {
                    leftWheelServo.setPosition(0.48);
                    rightWheelServo.setPosition(0.52);
                } else if (detectedColor.equals("YELLOW") || detectedColor.equals("RED")) {
                    leftWheelServo.setPosition(0.5);
                    rightWheelServo.setPosition(0.5);
                } else if (detectedColor.equals("BLUE") || detectedColor.equals("UNKNOWN")) {
                    leftWheelServo.setPosition(0.0);
                    rightWheelServo.setPosition(1.0);
                }

            } else if (gamepad2.left_stick_y < 0.0) {
                leftWheelServo.setPosition(1.0); // Full forward
                rightWheelServo.setPosition(0.0);
                //forWheel = 0.0;
            } else {
                leftWheelServo.setPosition(0.5);
                rightWheelServo.setPosition(0.5);
                //forWheel = 0.0;
            }

            // Optional: Add telemetry to display servo positions
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
