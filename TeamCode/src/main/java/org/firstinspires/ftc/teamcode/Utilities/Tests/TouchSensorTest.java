package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name="Touch Sensor Test", group="Tests")
public class TouchSensorTest extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Intake intake = new Intake();
    GamepadEx driverOp;

    @Override
    public void runOpMode() throws InterruptedException {

        intake.initTest(hardwareMap, driverOp);

        while(opModeInInit()){
            intake.getTelemetryTest(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){
            intake.getTelemetryTest(telemetry);
            telemetry.update();
        }
    }
}
