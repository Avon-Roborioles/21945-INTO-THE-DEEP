package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Subsystems.Computer_Vision;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.util.Drawing;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Specimen Alignment Test", group="Tests")
public class SpecimenAlignment extends LinearOpMode {
    PathChain alignPath;
    boolean blueColor = true;
    public enum States {
        ALIGN,
        HOLD
    }

    States currentState = States.HOLD;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    public void updateAuto(){
        bot.update(); //controls Pedro-Pathing logic

        vision.update();


        PoseStorage.CurrentPose = bot.getPose(); //updates currentPose variable
        mainTelemetry.addLine("---AUTO DATA---");
        mainTelemetry.addData("Auto Ran: ", PoseStorage.ranAuto);


        mainTelemetry.addData("Current State: ", currentState);
        mainTelemetry.addData("X Position: ", bot.getPose().getX());
        mainTelemetry.addData("Y Position: ", bot.getPose().getY());
        mainTelemetry.addData("Heading Position: ", bot.getPose().getHeading());
        mainTelemetry.addData("Total Heading: ", bot.getTotalHeading());
        vision.getTelemetry(mainTelemetry);

        //Draw Bot on Dashboard
        Drawing.drawPoseHistory(bot.getDashboardPoseTracker(), "#be87e8"); //Light Purple
        Drawing.drawRobot(bot.getPose(), "#8F2CDB"); //Main Purple
        Drawing.sendPacket();

        mainTelemetry.update();
    }

    //native wait method - milliseconds
    public void waitMilliSeconds(double milliseconds){
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(milliseconds >= time.time(TimeUnit.MILLISECONDS)){
            updateAuto();
            if(isStopRequested()){
                break;
            }
            if(!opModeIsActive()){
                break;
            }
        }
    }

    public void buildStrafePath(double distance){
        if(Math.abs(distance) < 10) {
            alignPath = bot.pathBuilder()
                    //align by moving forward and backward
                    .addPath(new BezierLine(bot.getPose().getPoint(), new Pose(bot.getPose().getX() -(distance + 3), bot.getPose().getY(), bot.getPose().getHeading()).getPoint()))
                    .setConstantHeadingInterpolation(bot.getPose().getHeading())
                    .build();
        } else {
            alignPath = bot.pathBuilder()
                    //align by moving forward and backward
                    .addPath(new BezierLine(bot.getPose().getPoint(), new Pose(bot.getPose().getX() + 0, bot.getPose().getY(), bot.getPose().getHeading()).getPoint()))
                    .setConstantHeadingInterpolation(bot.getPose().getHeading())
                    .build();
        }
    }

    Computer_Vision vision = new Computer_Vision();
    LED lighting = new LED();
    Follower bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Follower(hardwareMap);
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader a_button = new ToggleButtonReader(driverOp, GamepadKeys.Button.A);
        ToggleButtonReader b_button = new ToggleButtonReader(driverOp, GamepadKeys.Button.B);


        bot.setStartingPose(new Pose(0,0,0));

        lighting.init(hardwareMap);
        waitForStart();
        vision.init(hardwareMap);


        while(opModeIsActive()){
            switch (currentState){
                case HOLD:
                    if(!bot.isBusy()){
                        //logic
                        if(a_button.wasJustPressed()){ //blue alignment
                            vision.setAllianceColor(true);
                            blueColor = true;
                            waitMilliSeconds(500);
                            buildStrafePath(vision.getAlignment());
                            bot.followPath(alignPath,true);
                            currentState = States.ALIGN;

                        } else if(b_button.wasJustPressed()){ //red alignment
                            vision.setAllianceColor(false);
                            blueColor = false;
                            waitMilliSeconds(500);
                            buildStrafePath(vision.getAlignment());
                            bot.followPath(alignPath,true);
                            currentState = States.ALIGN;

                        }
                        break;
                    }
                case ALIGN:
                    if(!bot.isBusy()){
                        //logic
                        currentState = States.HOLD;
                        break;
                    }
            }
            if(blueColor){
                lighting.set(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
            } else {
                lighting.set(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            }
            a_button.readValue();
            b_button.readValue();
            updateAuto();
        }
    }
}
