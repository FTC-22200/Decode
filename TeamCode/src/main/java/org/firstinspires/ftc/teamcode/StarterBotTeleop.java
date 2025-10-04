package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
public class StarterBotTest extends OpMode {
    final double STOP_SPEED = 0.0;
    final double LAUNCHER_VELOCITY_THRESHOLD = 1500.00;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    double leftPower;
    double rightPower;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFeeder.setPower(STOP_SPEED);

        rightFeeder.setPower(STOP_SPEED);

        telemetry.addData("Status", "Initialized");
    }
    public void loop() {
        arcadeDrive(-gamepad1.right_stick_x, -gamepad1.left_stick_y);

        if (gamepad2.right_bumper) {
            juggle();
        } else {
            launch(-gamepad2.left_stick_y);
        }

        if (gamepad2.b) {
            leftFeeder.setPower(-1.0);
            rightFeeder.setPower(-1.0);
            launcher.setPower(-1.0);
        }
        if (launcher.getVelocity() <= LAUNCHER_VELOCITY_THRESHOLD && gamepad2.left_stick_y < 0.0) {
            if (gamepad2.x) {
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(-1.0);
            } else {
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
            }
        } else {
            if (gamepad2.a) { 
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(1.0);
            } else if (gamepad2.x) {
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(-1.0);
            } else {
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
            }
        }

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        telemetry.addData("Left Feeder Power", leftFeeder.getPower());
        telemetry.addData("Right Feeder Power", rightFeeder.getPower());

    }

    void arcadeDrive(double forward, double rotate) {
        rightPower = forward + rotate;
        leftPower = forward - rotate;

        if (gamepad1.left_trigger > 0.5) {
            leftDrive.setPower(leftPower*0.25);
            rightDrive.setPower(rightPower*0.25);
        } else {
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }
    }
    void launch(double power) {
        // Convert power to target velocity (assuming 1500 is max speed)
        double target_velocity = power * 1500.00;

        // Ensure the target velocity is within bounds
        target_velocity = Math.max(-1500.00, Math.min(target_velocity, 1500.00));

        // Set the motor velocity
        launcher.setVelocity(target_velocity);

        // Telemetry for debugging
        telemetry.addData("Launch Power", power);
        telemetry.addData("Target Velocity", target_velocity);
        telemetry.addData("Current Launcher Velocity", launcher.getVelocity());
    }
    void juggle() {
        double juggleVelocity = 625.0; // Adjust this value as needed
        launcher.setVelocity(juggleVelocity);

        telemetry.addData("Juggle Mode", "Active");
        telemetry.addData("Juggle Velocity", juggleVelocity);
    }

}
