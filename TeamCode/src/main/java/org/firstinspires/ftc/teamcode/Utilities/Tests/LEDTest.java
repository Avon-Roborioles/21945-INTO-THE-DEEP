package org.firstinspires.ftc.teamcode.Utilities.Tests;

//import needed
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.LED;

@TeleOp
public class LEDTest extends LinearOpMode {
    //creates lighting subsystem
    org.firstinspires.ftc.teamcode.Subsystems.LED lighting = new LED();

    //create input gamepad object
    GamepadEx driverOp;

    @Override
    public void runOpMode() throws InterruptedException{
        //assign driverOp to gamepad1
        driverOp = new GamepadEx(gamepad1);


        ButtonReader d_up = new ButtonReader(driverOp, GamepadKeys.Button.DPAD_UP);
        ButtonReader d_left = new ButtonReader(driverOp, GamepadKeys.Button.DPAD_LEFT);
        ButtonReader d_right = new ButtonReader(driverOp, GamepadKeys.Button.DPAD_RIGHT);
        ButtonReader d_down = new ButtonReader(driverOp, GamepadKeys.Button.DPAD_DOWN);
        ButtonReader b_button = new ButtonReader(driverOp, GamepadKeys.Button.B);
        ButtonReader x_button = new ButtonReader(driverOp, GamepadKeys.Button.X);
        ButtonReader y_button = new ButtonReader(driverOp, GamepadKeys.Button.Y);

        //initialize lighting
        lighting.init(hardwareMap);
        //start breathing mode (LED Status for Robot Idle)
        //lighting.setBreathingMode("RED");

        waitForStart();

        while(opModeIsActive()){
            //control lighting based on gamepad input


            //telemetry
            telemetry.addLine("> Press Dpad Up for Constant Purple");
            telemetry.addLine("> Press Dpad Left for Constant Red");
            telemetry.addLine("> Press Dpad Right for Constant Blue");
            telemetry.addLine("> Press Dpad Down for Off");
            telemetry.addLine("---------------");
            telemetry.addLine("> Press B for Breathing Red");
            telemetry.addLine("> Press X for Breathing Blue");
            telemetry.addLine("> Press Y for Constant Yellow");
            lighting.getTelemetry(telemetry);
            telemetry.update();

            //update button readers
            d_up.readValue();
            d_left.readValue();
            d_right.readValue();
        }

    }
}
