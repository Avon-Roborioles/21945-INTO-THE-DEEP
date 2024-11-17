package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Config
@TeleOp(name="Intake Auto Test", group="Tests")
public class IntakeAutoTest extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Intake intake = new Intake();
    GamepadEx driverOp;

    //0 is stop, 1 is pickup, 2 is drop
    public static int intakeControl = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        driverOp = new GamepadEx(gamepad1);

        intake.init(hardwareMap, driverOp);

        while(opModeInInit()){
            intake.getTelemetryBRIEF(telemetry);
        }

        waitForStart();

        while(opModeIsActive()){
            switch(intakeControl){
                case 0:
                    intake.stop();
                case 1:
                    intake.pickup();
                case 2:
                    intake.drop();
                default:
                    intake.stop();
            }
            
            intake.update(); //reads selected power and runs intake control
            intake.getTelemetryFULL(telemetry);
            telemetry.update();
        }
    }

}
