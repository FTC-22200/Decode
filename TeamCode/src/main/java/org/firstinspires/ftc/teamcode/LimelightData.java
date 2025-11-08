package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Limelight Data", group = "Test")
public class LimelightData extends OpMode {

    private Limelight3A limelight;
    private IMU imu;
    private double distance;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize Limelight and IMU
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // Switch to AprilTag pipeline

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("Initialization complete");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();

            telemetry.addData("Target Valid", true);
            telemetry.addData("Tx (Horizontal Offset)", llResult.getTx());
            telemetry.addData("Ty (Vertical Offset)", llResult.getTy());
            telemetry.addData("Ta (Target Area)", llResult.getTa());
            telemetry.addData("Bot Pose", botPose.toString());
        } else {
            telemetry.addData("Target Valid", false);
            telemetry.addLine("No valid AprilTag detected.");
        }

        telemetry.update();
    }
}
