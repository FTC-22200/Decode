package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.config.PIDConfig;


@Config
@TeleOp(name = "StarterBotTest", group = "StarterBot")
public class StarterBotTestPID extends OpMode {
    final double STOP_SPEED = 0.0;
    final double LAUNCHER_VELOCITY_THRESHOLD = 1300.00;
    double Servo_Time = 0.0;
    boolean Servo_Turning = false;
    double launcher_power = 1500.00;

    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private FtcDashboard dashboard;

    double leftPower;
    double rightPower;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFeeder.setPower(STOP_SPEED);

        rightFeeder.setPower(STOP_SPEED);

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        updatePIDCoefficients();
        telemetry.addData("Status", "Initialized");
    }
    public void loop() {
        arcadeDrive(-gamepad1.right_stick_x, -gamepad1.left_stick_y);

        if (gamepad2.right_bumper) {
            juggle();
        } else if (gamepad2.y) {
            launch();
        } else {
            launcher.setVelocity(0.0);
        }

        if (gamepad2.b) {
            leftFeeder.setPower(-1.0);
            rightFeeder.setPower(-1.0);
            launcher.setPower(-1.0);
        }
        if (launcher.getVelocity() <= LAUNCHER_VELOCITY_THRESHOLD && gamepad2.left_stick_y < 0.0) {
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
        } else {
            if (gamepad2.a && !Servo_Turning) {
                Servo_Time = 0.0;
                Servo_Turning = true;
            } else if (gamepad2.x) {
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(-1.0);
            } else {
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
            }
        }
        if (Servo_Turning) {
            if (Servo_Time < 8.0) {
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(1.0);
                Servo_Time++;
            } else {
                Servo_Turning = false;
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
    void launch() {
        // Convert power to target velocity (assuming 1500 is max speed)
        Servo_Time = 0.0;
        if (gamepad2.dpad_up) {
            launcher_power = 2500.00;
        } else if (gamepad2.dpad_left){
            launcher_power = 2000.00;
        } else if (gamepad2.dpad_right){
            launcher_power = 1500.00;
        }

        // Set the motor velocity
        launcher.setVelocity(launcher_power);
        telemetry.addData("co ", launcher.getVelocity() >= launcher_power - 50.0 && launcher.getVelocity() <= launcher_power + 50.0);
        if (launcher.getVelocity() >= launcher_power - 50.0 && launcher.getVelocity() <= launcher_power + 50.0) {
            //for (; Servo_Time < 24; Servo_Time++) {
            //    leftFeeder.setPower(1.0);
            //     rightFeeder.setPower(1.0);
            //     break; // Prevent locking up the loop in one cycle
            //}
            Servo_Turning = true;

        } else Servo_Time = 0.0;

        telemetry.addData("Target Velocity", launcher_power);
        telemetry.addData("Current Launcher Velocity", launcher.getVelocity());
    }
    void juggle() {
        double juggleVelocity = 625.0; // Adjust this value as needed
        launcher.setVelocity(juggleVelocity);

        telemetry.addData("Juggle Mode", "Active");
        telemetry.addData("Juggle Velocity", juggleVelocity);
    }
    private void updatePIDCoefficients() {
        PIDFCoefficients launcherPID = new PIDFCoefficients(
                PIDConfig.launcher_kP,
                PIDConfig.launcher_kI,
                PIDConfig.launcher_kD,
                PIDConfig.launcher_kF
        );
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launcherPID);
    }
}
