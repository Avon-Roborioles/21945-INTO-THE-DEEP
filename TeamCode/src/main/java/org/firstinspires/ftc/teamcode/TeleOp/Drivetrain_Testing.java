package org.firstinspires.ftc.teamcode.TeleOp;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.*;

@TeleOp(name="Drivetrain Testing")
public class Drivetrain_Testing extends LinearOpMode {

    //create drivetrain object
    private final org.firstinspires.ftc.teamcode.Subsystems.Drivetrain drivetrain = new Drivetrain();
    //FTClib object that encapsulates Gamepad 1 Controls with More Features
    GamepadEx Driver1Op = new GamepadEx(gamepad1);

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize drivetrain
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
