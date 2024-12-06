package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.util.Timing.Timer;
import org.firstinspires.ftc.teamcode.Subsystems.*;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Main TeleOp")
public class Main_TeleOp extends LinearOpMode {

    //create subsystem objects
    private final org.firstinspires.ftc.teamcode.Subsystems.Driver_Feedback feedback = new Driver_Feedback();
    private final org.firstinspires.ftc.teamcode.Subsystems.Drivetrain drivetrain = new Drivetrain();
    private final org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
    private final org.firstinspires.ftc.teamcode.Subsystems.Intake intake = new Intake();
    //private final org.firstinspires.ftc.teamcode.Subsystems.Computer_Vision vision = new Computer_Vision();
    //private final org.firstinspires.ftc.teamcode.Subsystems.LED lighting = new LED();

    //Driver gamepad objects - set to static so subsystems can access controls
    public static GamepadEx Driver1Op;
    public static GamepadEx Driver2Op;
    Timer opModeTimer = new Timer(120, TimeUnit.SECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        //FTClib object that encapsulates Gamepad 1 Controls with More Features
         Driver1Op = new GamepadEx(gamepad1);
         Driver2Op = new GamepadEx(gamepad2);

        //initialize subsystems
        feedback.init();
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap,Driver2Op, true);
        intake.init(hardwareMap, Driver2Op);
        //vision.init(hardwareMap);
        //lighting.init(hardwareMap);

        waitForStart();

        opModeTimer.start();

        while(opModeIsActive()){
            //Driver 1 Controls
            drivetrain.run_fieldCentric(Driver1Op, feedback);
            drivetrain.getTelemetryFULL(telemetry);


            //Driver 2 Controls
            arm.run_teleOp();
            intake.run_teleOp();

            //Telemetry
            drivetrain.getTelemetryBRIEF(telemetry);
            arm.getTelemetryBRIEF(telemetry);
            intake.getTelemetryBRIEF(telemetry);
            //vision.getTelemetryBRIEF(telemetry);
            //lighting.getTelemetryBRIEF(telemetry);
            telemetry.addData("OpMode Timer: ", opModeTimer.remainingTime());
            telemetry.update();
        }
    }
}
