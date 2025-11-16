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

        // Initialize Limelight and switch to AprilTag pipeline
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // Ensure pipeline 8 is configured for AprilTags

        // Initialize IMU with correct orientation
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
        // Update Limelight with current robot yaw
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        // Get latest Limelight result
        LLResult llResult = limelight.getLatestResult();

        // Check if result is valid and recent
        boolean isValid = llResult != null && llResult.isValid();

        // Adding telemetry data if target available
        telemetry.addData("Target Valid", isValid);

        if (isValid) {
            Pose3D botPose = llResult.getBotpose();
            double ta = llResult.getTa();
            double estimatedDistance = getDistanceFromTag(ta);
            // Telemetry Data - Distance
            telemetry.addData("Distance from BotPose (cm) - Second one", estimatedDistance);
            // Telemetry Data - Everything Else
            telemetry.addData("Tx (Horizontal Offset)", llResult.getTx());
            telemetry.addData("Ty (Vertical Offset)", llResult.getTy());
            telemetry.addData("Ta (Target Area)", llResult.getTa());
            telemetry.addData("Bot Pose", botPose.toString());
        } else {
            telemetry.addLine("No valid AprilTag detected.");
        }
        telemetry.update();
    }

    public double getDistanceFromTag(double ta) {
        if (ta <= 0) return -1;
        double a = 150.0;
        double b = -0.65;
        return a * Math.pow(ta, b); // distance returned in cm
    }
}