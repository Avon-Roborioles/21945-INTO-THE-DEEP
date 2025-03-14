package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Point;

@Autonomous(name="Auto Park", group = "Autos")
public class Park extends AutoBase {

    //important variables
    PathChain start, park;
    Pose startPose;
    Pose parkPose;
    Follower bot;
    AutoPoses AutoPose = getAutoPose();
    int inverseConstant = 1; //set to 1 or -1 depending on AutoPose - used only for parking
    GamepadEx driverOp;

    //Finite State Machine (FSM) variables
    public enum State {
        START,
        PARK,
        END
    }
    public State currentState;

    //add all paths/auto objectives here
    public void buildPaths(AutoPoses AutoPose){

        if(AutoPose == AutoPoses.LEFT){
            startPose = PoseStorage.LeftStartPose;
            parkPose = PoseStorage.LeftPark;
            inverseConstant = -8;

            start = bot.pathBuilder()
                    .addPath(new BezierLine(startPose.getPoint(), new Pose(parkPose.getX() + inverseConstant, parkPose.getY(),parkPose.getHeading()).getPoint()))
                    .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                    .build();

            park = bot.pathBuilder() //TODO use pathBuilder(bot) to test waitSeconds
                    .addPath(new BezierLine(new Pose(parkPose.getX() + inverseConstant, parkPose.getY(), parkPose.getHeading()).getPoint(), parkPose.getPoint()))
                    .setConstantHeadingInterpolation(parkPose.getHeading()) //sets constant heading for last path
                    .setPathEndVelocityConstraint(10) //sets constant velocity for last path
//                //.setPathEndTimeoutConstraint(3)
//                .waitSeconds(3) // custom waitSeconds method to make things easier - Test!!!
//
//                .addPath(new BezierLine(parkPose.getPoint(), new Pose(parkPose.getX() + inverseConstant, parkPose.getY(),parkPose.getHeading()).getPoint()))
//                //.setLinearHeadingInterpolation(parkPose.getHeading(), startPose.getHeading())
//                .addPath(new BezierLine(new Pose(parkPose.getX() + inverseConstant, parkPose.getY(), parkPose.getHeading()).getPoint(), startPose.getPoint()))
//                //.setConstantHeadingInterpolation(startPose.getHeading())
//                .setLinearHeadingInterpolation(parkPose.getHeading(), startPose.getHeading())
                    .build();

        } else if (AutoPose == AutoPoses.RIGHT) {
            startPose = PoseStorage.RightStartPose;
            parkPose = PoseStorage.RightPark;
            inverseConstant = 10;

            start = bot.pathBuilder()
                    .build();

            park = bot.pathBuilder() //TODO use pathBuilder(bot) to test waitSeconds
                    //drive to parkPose
                    .addPath(new BezierLine(startPose.getPoint(),parkPose.getPoint()))
                    .setConstantHeadingInterpolation(parkPose.getHeading())
                    .build();
        }

    };

    public void runOpMode() throws InterruptedException{
        bot = new Follower(hardwareMap);

        startPose = PoseStorage.LeftStartPose;
        parkPose = PoseStorage.LeftPark;

        driverOp = new GamepadEx(gamepad1);

        //initialize subsystems
        init_classes(driverOp);


        while(opModeInInit()){
            runMenu(telemetry);
            AutoPose = getAutoPose();

            telemetry.update();
        }

        waitForStart();
        PoseStorage.ranAuto = true;

        buildPaths(AutoPose); //builds paths after we select the autoStart pose from the menu

        bot.setPose(startPose);

       // starting path & FSM
        currentState = State.START;
        bot.followPath(start);


        while(opModeIsActive()){
            // FSM Auto Logic
            switch(currentState){
                case START:
                    if(!bot.isBusy()){
                        currentState = State.PARK;
                        bot.followPath(park);
                        break;
                    }
                case PARK:
                    if(!bot.isBusy()){
                        currentState = State.END;
                        bot.holdPoint(new BezierPoint(parkPose.getPoint()),parkPose.getHeading());
                        break;
                    }
            }

            bot.update(); //controls Pedro-Pathing logic
            PoseStorage.CurrentPose = bot.getPose(); //updates currentPose variable
            telemetry.addData("Selected Auto Position: ", AutoPose);
            telemetry.addData("Selected Park Position: ", AutoPose);
            telemetry.addData("Inverse Constant: ", inverseConstant);
            telemetry.addData("Current State: ", currentState);
            telemetry.addData("X Position: ", bot.getPose().getX());
            telemetry.addData("Y Position: ", bot.getPose().getY());
            telemetry.addData("Heading Position: ", bot.getPose().getHeading());
            telemetry.update();
        }
    }
}