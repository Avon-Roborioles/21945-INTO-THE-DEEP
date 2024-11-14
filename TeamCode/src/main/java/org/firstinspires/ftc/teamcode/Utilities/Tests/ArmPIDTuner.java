package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;


@TeleOp(name="Arm PID Tuner", group="Tests")
public class ArmPIDTuner extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
    GamepadEx driverOp = new GamepadEx(gamepad1);

    //TODO - tune these PID values & Paste to Arm Subsystem Class
    //Pulled from Arm Subsytem to directly tune there
    public double p = Arm.p;
    public double i = Arm.i;
    public double d = Arm.d;
    public double f = Arm.f;

    //position variable to test PID movement
    public int target = Arm.target;

    //check armMotor to be accurate
    private final double ticks_in_degree = 700 / 180.0;


    @Override
    public void runOpMode() throws InterruptedException {
        //init
        arm.initPID(hardwareMap, driverOp);

        while(opModeInInit()){
            arm.setupEMotor(); //pulls extension in
            arm.getTelemetryFULL(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){
            arm.set_arm(target);

            arm.update(); //updates PID control
            arm.getTelemetryFULL(telemetry);
            telemetry.update();
        }

    }
}
