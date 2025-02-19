package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Path;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Test Auto", group = "Autos")
public class Test_Auto extends AutoBase{
    //important variables
    Path testPath;
    Pose startPose;
    Pose parkPose;
    Follower bot;
    AutoPoses AutoPose = getAutoPose();
    GamepadEx driverOp;
    String message = "";

    public int pathState = 0;

    public void buildPaths(){
        //testPath = new
    }

    public void waitSeconds(double seconds){
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(seconds >= time.time(TimeUnit.SECONDS)){
            updateAuto();
            if(isStopRequested()){
                break;
            }
            if(!opModeIsActive()){
                break;
            }
        }
    }

    //various PID updating & Telemetry Data bundled into one method
    public void updateAuto(){
        bot.update(); //controls Pedro-Pathing logic
        //TODO - subsystemsUpdate();
        PoseStorage.CurrentPose = bot.getPose(); //updates currentPose variable
        telemetry.addData("Selected Auto Position: ", AutoPose);
        telemetry.addData("Selected Park Position: ", AutoPose);
        telemetry.addData("X Position: ", bot.getPose().getX());
        telemetry.addData("Y Position: ", bot.getPose().getY());
        telemetry.addData("Heading Position: ", bot.getPose().getHeading());
        telemetry.addData("Message: ", message);
        telemetry.update();
    }

    public void runOpMode() throws InterruptedException {
        bot = new Follower(hardwareMap);
        startPose = PoseStorage.LeftStartPose;
        parkPose = PoseStorage.LeftPark;

        driverOp = new GamepadEx(gamepad1);

        //initialize subsystems
        init_classes(driverOp);


        while (opModeInInit()) {
            runMenu(telemetry);
            AutoPose = getAutoPose();
            telemetry.update();
        }

        waitForStart();

        buildPaths(); //builds paths after we select the autoStart pose from the menu

        bot.setPose(startPose);

        //follow paths here
        switch (pathState){
            case 0:
                if (!bot.isBusy()) {
                    //followPath
                    //set State
                    break;
                }
            case 1:
                if (!bot.isBusy()) {
                    //followPath
                    //set State
                    break;
                }
        }

        updateAuto();
    }
}
