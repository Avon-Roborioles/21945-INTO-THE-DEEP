package org.firstinspires.ftc.teamcode.TeleOp;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.*;

//@TeleOp(name="Field-Centric TeleOp")
@Disabled
public class Field_Centric_TeleOp extends LinearOpMode {

    //create drivetrain object
    private final org.firstinspires.ftc.teamcode.Subsystems.Drivetrain drivetrain = new Drivetrain();
    private final org.firstinspires.ftc.teamcode.Subsystems.Driver_Feedback feedback = new Driver_Feedback();
    //FTClib object that encapsulates Gamepad 1 Controls with More Features

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize drivetrain

        GamepadEx Driver1Op = new GamepadEx(gamepad1);

        drivetrain.init(hardwareMap, Driver1Op);
        feedback.init();

        waitForStart();

        //run drivetrain software in loop
        while(opModeIsActive()){
            drivetrain.run_fieldCentric();
            drivetrain.getTelemetryFULL(telemetry);
            telemetry.update();
        }


    }
}
