package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;


@TeleOp(name="Arm PID Tuner", group="Tests")
public class ArmPIDTuner extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
    GamepadEx driverOp;


    //position variable to test PID movement
    public int armTarget = Arm.armTarget;
    public int extendTarget = Arm.extendTarget;


    @Override
    public void runOpMode() throws InterruptedException {

        driverOp = new GamepadEx(gamepad1);

        //init
        arm.initPID(hardwareMap, driverOp);

        while(opModeInInit()){
            arm.setupEMotor(); //pulls extension in
            arm.getTelemetryFULL(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){
            arm.set_pose(armTarget);
            //switch to extend after turning arm
            //arm.extend(extendTarget);

            arm.update(); //updates PID control
            arm.getTelemetryFULL(telemetry);
            telemetry.update();
        }

    }
}
