package org.firstinspires.ftc.teamcode.Autonomous.Left_Autos;

//import needed libraires
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Autonomous.Right_Autos.Right_Park;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.PoseStoragePedro;


@Autonomous(name="Park Auto", group = "Left Autos")
public class Left_Park extends AutoBase {

    //important variables
    PathChain start, park;
    Pose startPose = PoseStoragePedro.LeftStartPose;
    Pose parkPose = PoseStoragePedro.LeftPark;
    Follower bot;

    //Finite State Machine (FSM) variables
    public enum State {
        START,
        PARK,
        END
    }

    public State currentState;


    //add all paths/auto objectives here
    public void buildPaths(){
        start = bot.pathBuilder()
                .addPath(new BezierLine(startPose.getPoint(), new Pose(parkPose.getX()-15, parkPose.getY(),parkPose.getHeading()).getPoint()))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .build();

        park = bot.pathBuilder()
                .addPath(new BezierLine(new Pose(parkPose.getX()-13, parkPose.getY(), parkPose.getHeading()).getPoint(), parkPose.getPoint()))
                .setConstantHeadingInterpolation(parkPose.getHeading()) //sets constant heading for last path
                .setPathEndVelocityConstraint(10) //sets constant velocity for last path
                //.setPathEndTimeoutConstraint(3)
                .waitSeconds(3) //TODO custom waitSeconds method to make things easier - Test!!!

                .addPath(new BezierLine(parkPose.getPoint(), new Pose(parkPose.getX()-15, parkPose.getY(),parkPose.getHeading()).getPoint()))
                .setLinearHeadingInterpolation(parkPose.getHeading(), startPose.getHeading())
                .addPath(new BezierLine(new Pose(parkPose.getX()-15, parkPose.getY(), parkPose.getHeading()).getPoint(), startPose.getPoint()))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    };

    public void runOpMode() throws InterruptedException{
        bot = new Follower(hardwareMap);

        //vision.init_sample_detection(SAMPLE.COLOR - ALLIANCE COLOR)

        bot.setPose(startPose);

        //initialize subsystems
        init_classes();

        //TODO - vision.init_sample_detection(SAMPLE.COLOR - YELLOW)

        //TODO - Test
        //        while(opModeInInit()){
        //            runMenu();
        //            telemetry.update();
        //        }


        waitForStart();

        buildPaths();

        //starting path & FSM
        currentState = State.START;
        bot.followPath(start);

        while(opModeIsActive()){
            //FSM Auto Logic
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
            telemetry.addData("Current State: ", currentState);
            telemetry.addData("X Position: ", bot.getPose().getX());
            telemetry.addData("Y Position: ", bot.getPose().getY());
            telemetry.addData("Heading Position: ", bot.getPose().getHeading());
            telemetry.update();

        }



    }

}
