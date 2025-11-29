package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.hardware.rev.RevBlinkinLedDriver;



@TeleOp
public class DecodeDriveMode extends LinearOpMode {
    double launcher_velocity = 3000.0;
    boolean boxServoUp = false;
    private ElapsedTime boxServoTimer = new ElapsedTime();
    private DcMotor intakeMotor;
    private DcMotorEx launcher;
    Servo rgbLight;

    @Override
    public void runOpMode() {
        ColorSensor colorSensor;
        // Motor config
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        rgbLight = hardwareMap.get(Servo.class, "RGB Light");

        // Motor directions
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.FORWARD);

        final int CYCLE_MS = 50;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Color declaration
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();
            int purple = colorSensor.red() + colorSensor.blue();
            telemetry.addData("Green", green);
            telemetry.addData("Purple", purple);
            telemetry.addData("Red", red);
            telemetry.addData("Blue", blue);

            if (gamepad2.right_trigger > 0) {
                launch();
            } else {
                launcher.setPower(0.0);
            }

            double speedMultiplier = 1.0;
            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier *= 0.4; // Original - prev was 0.5
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

            // To decrease 'noise' via small movements
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(y) < 0.05) y= 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double fL_Motor = (y + x + rx) / denominator;
            double bL_Motor = (y - x + rx) / denominator;
            double fR_Motor = (y - x - rx) / denominator;
            double bR_Motor = (y + x - rx) / denominator;

            frontLeft.setPower(fL_Motor);
            backLeft.setPower(bL_Motor);
            frontRight.setPower(fR_Motor);
            backRight.setPower(bR_Motor);

            // Intake motor control
            if (gamepad2.right_stick_y != 0.0) {
                intakeMotor.setPower(gamepad2.right_stick_y);
            } else {
                intakeMotor.setPower(0.0);
            }

            // Box servo to push the ball into the box
            if (gamepad2.y && launcher.getVelocity() >= launcher_velocity && !boxServoUp) {
                boxServo.setPosition(0.6);
                boxServoUp = true;
                boxServoTimer.reset();
            }
            // Timer
            if (boxServoUp && boxServoTimer.seconds() > 0.75) {
                boxServo.setPosition(0.85);
                boxServoUp = false;
            }

            String detectedColor = "UNKNOWN";
            if (red > green && red > blue || green < 250 && purple < 250) {
                detectedColor = "WHITE";
            } else if (green > blue && green > red) {
                detectedColor = "GREEN";
            } else if (green < purple) {
                detectedColor = "PURPLE";
            }
            telemetry.addData("Detected Color:", detectedColor);

            // Color conditions and led lightup
            /*if (green > purple && green > 50) {
                detectedColor = "GREEN";
            } else if (purple > green && purple > 100) {
                detectedColor = "PURPLE";
            } else {
                detectedColor = "NULL";
            }
             */

            // Problem: Green and Purple show, however null does not show
            if (detectedColor.equals("GREEN")) {
                rgbLight.setPosition(0.500);
            } else if (detectedColor.equals("PURPLE")) {
                rgbLight.setPosition(0.7222);
            } else {
                rgbLight.setPosition(1.0);
            }

            //Incremental velocity power
            if (gamepad2.left_stick_y > 0.0) {
                launcher_velocity += 100;
            } else if (gamepad2.left_stick_y < 0.0) {
                launcher_velocity -= 100;
            }

            telemetry.update();
            telemetry.addData("Launcher target velocity : ", launcher_velocity);
            telemetry.addData("Acc target velocity: ", launcher.getVelocity());
            sleep(CYCLE_MS);
            idle();
        }
    }
    public void launch() { // changed from private
        if (gamepad2.dpad_up) { // high
            launcher_velocity = 2600.0; // originally 2700.0
        } else if (gamepad2.dpad_left) { // medium
            launcher_velocity = 2400.0;
        } else if (gamepad2.dpad_right) { // low-mid (new)
            launcher_velocity = 1800.0;
        } else if (gamepad2.dpad_down) { // low
            launcher_velocity = 1200.0 ;
        }
        launcher.setVelocity(launcher_velocity);
    }
}
