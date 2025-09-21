package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleop_NoEncoder", group = "StarterBot")
public class StarterBotTest extends OpMode {
    final double FEED_TIME_SECONDS = 0.20;
    final double SPIN_UP_TIME_SECONDS = 1.0;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 10.0;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime spinUpTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    double leftPower;
    double rightPower;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
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

    @Override
    public void loop() {
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Manual launcher control
        if (gamepad2.y) {
            launcher.setPower(FULL_SPEED);
        } else if (gamepad2.b) {
            launcher.setPower(STOP_SPEED);
        }

        // Manual feeder test (press A to activate feeders)
        if (gamepad2.a) {
            leftFeeder.setPower(1.0);
            rightFeeder.setPower(1.0);
        } else {
            leftFeeder.setPower(0.0);
            rightFeeder.setPower(0.0);
        }

        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Feeder Timer", feederTimer.seconds());
        telemetry.addData("SpinUp Timer", spinUpTimer.seconds());
        telemetry.addData("Left Feeder Power", leftFeeder.getPower());
        telemetry.addData("Right Feeder Power", rightFeeder.getPower());

    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}
