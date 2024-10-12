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
//    private final org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
//    private final org.firstinspires.ftc.teamcode.Subsystems.Intake intake = new Intake();
//    private final org.firstinspires.ftc.teamcode.Subsystems.Lift lift = new Lift();
//    private final org.firstinspires.ftc.teamcode.Subsystems.Computer_Vision vision = new Computer_Vision();
//    private final org.firstinspires.ftc.teamcode.Subsystems.LED lighting = new LED();
    private final org.firstinspires.ftc.teamcode.Subsystems.Driver_Feedback feedback = new Driver_Feedback();

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
        feedback.init();

        waitForStart();

        //run drivetrain software in loop
        while(opModeIsActive()){
            drivetrain.run_fieldCentric(Driver1Op, feedback);
            drivetrain.getTelemetryFULL(telemetry);

            telemetry.addLine("---Endgame Timer---");
            telemetry.addData("Time Left: ", feedback.getTimer());
            telemetry.update();
        }
    }
}
