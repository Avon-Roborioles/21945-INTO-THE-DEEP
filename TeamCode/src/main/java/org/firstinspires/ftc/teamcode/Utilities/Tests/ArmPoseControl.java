package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@Config
@TeleOp(name="Arm Pose Control", group="Tests")
public class ArmPoseControl extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
    GamepadEx driverOp;


    //position variable to test PID movement
    public static int armTarget = 0;
    public int extendTarget = Arm.extendTarget;
    public int count = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        driverOp = new GamepadEx(gamepad1);

        //init
        //arm.initPID(hardwareMap, driverOp);
        arm.initPoseControl(hardwareMap, driverOp);

//        while(opModeInInit()){
//            arm.setupEMotor(); //pulls extension in
//            arm.getTelemetryFULL(telemetry);
//            telemetry.update();
//        }

        waitForStart();

        arm.set_pose_Main(1);
        count++;

        while(opModeIsActive()){
           // arm.set_pose_Main(armTarget);
            telemetry.addData("Pose Count: ", count);
            arm.getTelemetryFULL(telemetry);
            telemetry.update();
        }

    }
}
