package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Subsystems.*;

import java.util.concurrent.TimeUnit;

//@Photon
@TeleOp(name="TeleOp w/ No Slides")
public class V1_TeleOp extends AutoBase {

    //create subsystem objects
    private final org.firstinspires.ftc.teamcode.Subsystems.Driver_Feedback feedback = new Driver_Feedback();
    private final org.firstinspires.ftc.teamcode.Subsystems.Drivetrain drivetrain = new Drivetrain();
    private final org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
    private final org.firstinspires.ftc.teamcode.Subsystems.Intake intake = new Intake();
    //private final org.firstinspires.ftc.teamcode.Subsystems.LED lighting = new LED();

    MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        drivetrain.init(hardwareMap, Driver1Op);
        arm.init(hardwareMap,Driver2Op, true);
        intake.init(hardwareMap, Driver2Op);
        //vision.init(hardwareMap);
        //lighting.init(hardwareMap);

        initTeleOpMenu(Driver1Op);

        while(opModeInInit()){
            runTeleOpMenu(mainTelemetry);
        }

        waitForStart();

        opModeTimer.start();

        while(opModeIsActive()){
            //Driver 1 Controls
            drivetrain.run_teleOp(feedback);

            //Driver 2 Controls
            arm.run_teleOp(feedback);
            intake.run_teleOp();

            //endgame alert to drivers
            if(opModeTimer.remainingTime() == 30){
                feedback.alert_drivers(Driver1Op,Driver2Op);
            }

            //Telemetry
            drivetrain.getTelemetryBRIEF(mainTelemetry);
            arm.getTelemetry(mainTelemetry);
            intake.getTelemetryFULL(mainTelemetry);
            //lighting.getTelemetryBRIEF(telemetry);
            mainTelemetry.addData("OpMode Timer: ", opModeTimer.remainingTime());
            mainTelemetry.update();
        }
    }
}
