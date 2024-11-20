package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name="Intake TeleOp Test", group = "Tests")
public class IntakeTeleOpTest extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Intake intake = new Intake();
    GamepadEx driverOp;


    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad1);

        intake.init(hardwareMap, driverOp);

        while(opModeInInit()){
            intake.getTelemetryBRIEF(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){
            intake.run_teleOp();
            intake.getTelemetryFULL(telemetry);
            telemetry.update();
        }
    }
}
