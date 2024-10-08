package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.*;

@TeleOp(name="Main TeleOp")
public class Main_TeleOp extends LinearOpMode {

    //create subsystem objects
    private final org.firstinspires.ftc.teamcode.Subsystems.Drivetrain drivetrain = new Drivetrain();

    //Driver gamepad objects - set to static so subsystems can access controls
    public static GamepadEx Driver1Op;
    public static GamepadEx Driver2Op;

    @Override
    public void runOpMode() throws InterruptedException {
        //FTClib object that encapsulates Gamepad 1 Controls with More Features
         Driver1Op = new GamepadEx(gamepad1);
         Driver2Op = new GamepadEx(gamepad2);

        //initialize subsystems
        drivetrain.init(hardwareMap);

        waitForStart();

        //run drivetrain software in loop
        while(opModeIsActive()){
            drivetrain.run_fieldCentric(Driver1Op);
            drivetrain.getTelemetryFULL(telemetry);
            telemetry.update();
        }
    }
}
