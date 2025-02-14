package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Driver_Feedback;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOp w/ Lift")
//@Disabled
public class V2_TeleOp extends AutoBase {
    //create subsystem objects
    private final org.firstinspires.ftc.teamcode.Subsystems.Driver_Feedback feedback = new Driver_Feedback();
    private final org.firstinspires.ftc.teamcode.Subsystems.Drivetrain drivetrain = new Drivetrain();
    private final org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
    private final org.firstinspires.ftc.teamcode.Subsystems.Lift lift = new Lift();
    private final org.firstinspires.ftc.teamcode.Subsystems.Intake intake = new Intake();
    private final org.firstinspires.ftc.teamcode.Subsystems.LED lighting = new LED();

    MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    //Driver gamepad objects - set to static so subsystems can access controls
    public static GamepadEx Driver1Op;
    public static GamepadEx Driver2Op;
    Timing.Timer opModeTimer = new Timing.Timer(120, TimeUnit.SECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        //FTClib object that encapsulates Gamepad 1 Controls with More Features
        Driver1Op = new GamepadEx(gamepad1);
        Driver2Op = new GamepadEx(gamepad2);

        //initialize subsystems
        feedback.init();
        drivetrain.init(hardwareMap, Driver1Op);
        arm.init(hardwareMap,Driver2Op, true);
        lift.init(hardwareMap,Driver2Op,true);
        intake.init(hardwareMap, Driver2Op);
        //vision.init(hardwareMap); - TODO useful for Edgar's specimen alignment feature
        lighting.init(hardwareMap);

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
            lift.run_teleOp(feedback);

            //lights
            lighting.setConstantColor(LED.COLORS.PURPLE);

            //endgame alert to drivers
            if(opModeTimer.remainingTime() == 30){
                feedback.alert_drivers(Driver1Op,Driver2Op);
            }

            //Telemetry
            mainTelemetry.addData("Auto Ran: ", PoseStorage.ranAuto);
            drivetrain.getTelemetryBRIEF(mainTelemetry);
            arm.getTelemetry(mainTelemetry);
            intake.getTelemetryFULL(mainTelemetry);
            lift.getTelemetry(telemetry);
            lighting.getTelemetry(telemetry);
            mainTelemetry.addData("OpMode Timer: ", opModeTimer.remainingTime());
            mainTelemetry.update();
        }
    }

}
