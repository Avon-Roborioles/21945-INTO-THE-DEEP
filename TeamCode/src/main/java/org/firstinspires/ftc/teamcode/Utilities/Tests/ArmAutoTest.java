package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@Config
@TeleOp(name="Arm Auto Test", group="Tests")
public class ArmAutoTest extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
    GamepadEx driverOp;


    //position variable to test PID movement
    public static int armTarget = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad1);

        arm.initNEW(hardwareMap, driverOp,false);

        while(opModeInInit()){
            arm.getTelemetryBRIEF(telemetry);
        }

        waitForStart();

        while(opModeIsActive()){
            //arm.goTo(armTarget);
            arm.setTarget(armTarget);
            arm.update();
            arm.getTelemetryFULL(telemetry);
            telemetry.update();
        }

    }
}
