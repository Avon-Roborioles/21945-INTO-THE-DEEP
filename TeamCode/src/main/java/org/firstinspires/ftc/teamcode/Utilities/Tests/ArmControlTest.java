package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class ArmControlTest extends LinearOpMode {

    private final org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();

    public static GamepadEx driver1Op;


    @Override
    public void runOpMode() throws InterruptedException {

        driver1Op = new GamepadEx(gamepad1);

        arm.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            //run basic auto control
            arm.run_teleOpBASIC(driver1Op);

            arm.getTelemetryFULL(telemetry);
            telemetry.update();
        }

    }
}
