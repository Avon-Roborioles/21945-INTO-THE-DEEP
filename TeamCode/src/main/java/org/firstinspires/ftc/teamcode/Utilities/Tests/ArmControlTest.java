package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@TeleOp(name="Arm Control Test", group = "Tests")
public class ArmControlTest extends LinearOpMode {

    private final org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();

    public static GamepadEx driver1Op;


    @Override
    public void runOpMode() throws InterruptedException {

        driver1Op = new GamepadEx(gamepad1);

        arm.init(hardwareMap, driver1Op);

        while(opModeInInit()){
            arm.setupEMotor();
            arm.getTelemetryBRIEF(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){
            //run basic auto control
            arm.run_teleOpBASIC();

            arm.getTelemetryFULL(telemetry);
            telemetry.update();
        }

    }
}
