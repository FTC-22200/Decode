package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Autonomous(name="DecodeAuto", group="Decode")
//@Disabled
public class DecodeAuto extends OpMode
{
    private DcMotor intakeMotor;
    private DcMotorEx launcher;
    double launcher_power = 1.0;
    double launcher_velocity = 3000.0;
    private CRServo boxServo = null;
    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH;
    }
    private LaunchState launchState;
    private enum AutoState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE;
    }
    private AutoState autoState;

    private enum Alliance {
        RED,
        Blue;
    }

    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        autoState = AutoState.LAUNCH;
        launchState = LaunchState.IDLE;

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
    }
    public void loop() {

    }
}


