package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.PoseStoragePedro;

@Autonomous(name="Right Park", group = "Right Autos")
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
            startPose = PoseStoragePedro.LeftStartPose;
            parkPose = PoseStoragePedro.LeftPark;
            inverseConstant = -4;

        } else {
            startPose = PoseStoragePedro.RightStartPose;
            parkPose = PoseStoragePedro.RightPark;
            inverseConstant = 10;
        }

        start = bot.pathBuilder()
                .addPath(new BezierLine(startPose.getPoint(), new Pose(parkPose.getX() + inverseConstant, parkPose.getY(),parkPose.getHeading()).getPoint()))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .build();

        park = bot.pathBuilder() //TODO use pathBuilder(bot) to test waitSeconds
                .addPath(new BezierLine(new Pose(parkPose.getX() + inverseConstant, parkPose.getY(), parkPose.getHeading()).getPoint(), parkPose.getPoint()))
                .setConstantHeadingInterpolation(parkPose.getHeading()) //sets constant heading for last path
                .setPathEndVelocityConstraint(10) //sets constant velocity for last path
                //.setPathEndTimeoutConstraint(3)
                .waitSeconds(3) //TODO custom waitSeconds method to make things easier - Test!!!

                .addPath(new BezierLine(parkPose.getPoint(), new Pose(parkPose.getX() + inverseConstant, parkPose.getY(),parkPose.getHeading()).getPoint()))
                //.setLinearHeadingInterpolation(parkPose.getHeading(), startPose.getHeading())
                .addPath(new BezierLine(new Pose(parkPose.getX() + inverseConstant, parkPose.getY(), parkPose.getHeading()).getPoint(), startPose.getPoint()))
                //.setConstantHeadingInterpolation(startPose.getHeading())
                .setLinearHeadingInterpolation(parkPose.getHeading(), startPose.getHeading())
                .build();

    };

    public void runOpMode() throws InterruptedException{
        bot = new Follower(hardwareMap);

        //vision.init_sample_detection(SAMPLE.COLOR - ALLIANCE COLOR)
        startPose = PoseStoragePedro.LeftStartPose;
        parkPose = PoseStoragePedro.LeftPark;

        driverOp = new GamepadEx(gamepad1);

        //initialize subsystems
        init_classes(driverOp);

        //TODO - vision.init_sample_detection(SAMPLE.COLOR - YELLOW)


        //TODO - Test
        while(opModeInInit()){
            runMenu();
            AutoPose = getAutoPose();

            telemetry.update();
        }


        waitForStart();

        buildPaths(AutoPose); //should build paths after we select the autoStart pose from the mennu

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
                        break;
                    }
            }

            bot.update(); //controls Pedro-Pathing logic
            PoseStoragePedro.CurrentPose = bot.getPose(); //updates currentPose variable
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