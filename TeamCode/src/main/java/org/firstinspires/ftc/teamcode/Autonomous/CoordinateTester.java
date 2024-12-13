package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.PoseStoragePedro;

@Autonomous(name="Coordinate Tester", group = "Autos")
public class CoordinateTester extends AutoBase{

    //important variables
    PathChain startToTarget, targetToStart;
    Pose startPose, targetPose;
    String targetPoseName;
    Follower bot;
    AutoPoses AutoPose;
    boolean returnHome;
    GamepadEx driverOp;
    ToggleButtonReader a_button;

    //Finite State Machine (FSM) variables
    public enum State {
        START, //go to target
        RETURN, //return to startPose
        END //end of program
    }

    public State currentState;

    //add all paths/auto objectives here
    public void buildPaths(AutoPoses AutoPose){
        if(AutoPose == AutoPoses.LEFT){
            startPose = PoseStoragePedro.LeftStartPose;
        } else if (AutoPose == AutoPoses.RIGHT) {
            startPose = PoseStoragePedro.RightStartPose;
        }

        startToTarget = bot.pathBuilder()
                .addPath(new BezierLine(startPose.getPoint(), targetPose.getPoint()))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();

        targetToStart = bot.pathBuilder()
                .addPath(new BezierLine(targetPose.getPoint(), startPose.getPoint()))
                .setLinearHeadingInterpolation(targetPose.getHeading(), startPose.getHeading())
                .build();
    }

    public void runOpMode() throws InterruptedException{
        bot = new Follower(hardwareMap);

        startPose = PoseStoragePedro.LeftStartPose;
        targetPose = Coordinates[0];
        targetPoseName = CoordinateNames[0];

        driverOp = new GamepadEx(gamepad1);

        //initialize subsystems
        init_classes(driverOp);

        //TODO - create toggleReader for returning back to start
        a_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );

        //run Auto Menu
        while(opModeInInit()){
            runCoordinateMenu();
            AutoPose = getAutoPose();
            targetPose = getSelectedCoordinate();
            targetPoseName = getSelectedCoordinateName();
            returnHome = getReturnHome();
            telemetry.update();
        }

        waitForStart();

        buildPaths(AutoPose);

        bot.setPose(startPose);

        //starting path & FSM
        currentState = State.START;
        bot.followPath(startToTarget);

        while(opModeIsActive()){
            //FSM Auto Logic
            //arm.runPassiveExtend();
            switch(currentState){
                case START:
                    if(!bot.isBusy()){
                        bot.holdPoint(new BezierPoint(targetPose.getPoint()), targetPose.getHeading());
                        if(a_button.wasJustPressed() && returnHome) {
                            currentState = State.RETURN;
                            bot.followPath(targetToStart);
                        }
                        break;
                    }
                case RETURN:
                    if(!bot.isBusy()) {
                        currentState = State.END;
                    }
            }


            bot.update(); //controls Pedro-Pathing logic
            PoseStoragePedro.CurrentPose = bot.getPose(); //updates currentPose variable
            telemetry.addData("Selected Auto Position: ", AutoPose);
            telemetry.addData("Selected Target Position: ", targetPoseName);
            telemetry.addData("Current State: ", currentState);
            telemetry.addData("X Position: ", bot.getPose().getX());
            telemetry.addData("Y Position: ", bot.getPose().getY());
            telemetry.addData("Heading Position: ", bot.getPose().getHeading());
            telemetry.addData("A Button State: ", a_button.getState());
            telemetry.update();
            a_button.readValue(); //updates a_button reader
        }
    }
}
