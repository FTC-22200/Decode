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

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    double leftPower;
    double rightPower;
    double power;

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
        launch(-gamepad2.left_stick_y);

        // Manual feeder control
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
        launcher.setPower(power);
    }
}
