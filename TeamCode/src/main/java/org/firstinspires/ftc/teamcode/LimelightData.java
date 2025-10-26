package org.firstinspires.ftc.teamcode;
// Imports for limelight, OpMode for teleop, etc.
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Code/file used for AprilTag Detection and Measuring distance between bot and tag

public class LimelightData extends OpMode {

    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    // private DcMotorEx launcher = null;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // April Tag pipeline
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

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
            // Data with Control Hub presented
            distance = getDistance(llResult.getTa());
            telemetry.addData("Tx", llResult.getTx()); // Horizontal Offset (left/right)
            telemetry.addData("Ty", llResult.getTy()); // Vertical Offset (up/down)
            telemetry.addData("Ta", llResult.getTa()); // Target Area (sort of center)
            telemetry.addData("botPose", botPose.toString()); // Position of robot
            telemetry.addData("Distance from Tag", distance); // Distance from robot to tag
        }
    }


    // Code for measuring distance
    public double getDistance(double ta) {
        double scale = 4000.0; // Test (Still need to get data)
        distance = (scale / ta);
        return distance;
    }
}

