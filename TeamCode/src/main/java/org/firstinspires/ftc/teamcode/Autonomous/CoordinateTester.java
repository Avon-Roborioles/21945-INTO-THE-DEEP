package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;

@Config
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
    public int prevArmTarget = 0;
    public static int armTarget = 0;
    public int prevExtendTarget = 0;
    public static int extendTarget = 0;

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
            startPose = PoseStorage.LeftStartPose;
        } else if (AutoPose == AutoPoses.RIGHT) {
            startPose = PoseStorage.RightStartPose;
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

        startPose = PoseStorage.LeftStartPose;
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

            if(prevArmTarget != armTarget){
                prevArmTarget = armTarget;
                arm.setTarget(armTarget,extendTarget);
            }
            if(prevExtendTarget != extendTarget){
                prevExtendTarget = extendTarget;
                arm.setTarget(armTarget,extendTarget);
            }

            bot.update(); //controls Pedro-Pathing logic
            arm.update();

            PoseStorage.CurrentPose = bot.getPose(); //updates currentPose variable
            telemetry.addData("Selected Auto Position: ", AutoPose);
            telemetry.addData("Selected Target Position: ", targetPoseName);
            telemetry.addData("Current State: ", currentState);
            telemetry.addData("X Position: ", bot.getPose().getX());
            telemetry.addData("Y Position: ", bot.getPose().getY());
            telemetry.addData("Heading Position: ", bot.getPose().getHeading());
            telemetry.addData("A Button State: ", a_button.getState());
            arm.getTelemetry(telemetry);

            telemetry.update();
            a_button.readValue(); //updates a_button reader
        }
    }
}
