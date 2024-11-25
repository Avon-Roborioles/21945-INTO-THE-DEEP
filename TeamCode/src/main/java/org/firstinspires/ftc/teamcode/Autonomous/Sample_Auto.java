package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Utilities.PoseStoragePedro;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.util.Timer;

import java.util.concurrent.TimeUnit;

@Autonomous(name="3 Sample Auto", group = "Autos")
public class Sample_Auto extends AutoBase {
    //important variables
    Path scorePassive, Sample1, Sample2, Sample3, score, park, returnToCheckPoint;
    Pose startPose;
    Pose parkPose;
    Follower bot;
    AutoPoses AutoPose = getAutoPose();
    int inverseConstant = 1; //set to 1 or -1 depending on AutoPose - used only for parking
    GamepadEx driverOp;
    Timer opModeTimer;
    Timer waitTimer;
    Thread pathingThread;
    Thread subsystemThread;
    String message = "";



    //Finite State Machine (FSM) variables
    public enum State {
        SCORE_PASSIVE,
        GET_GROUND_SAMPLE,
        RETURN_TO_CHECKPOINT,
        SCORE,
        PARK,
        END
    }

    public State currentState;
    public int groundSamplesScored = 0;


    //add all paths/auto objectives here
    //TODO make sure all main paths return to LeftCheckPoint
    //TODO make sure all arm commands eventually go down in each pathChain
    public void buildPaths(AutoPoses AutoPose) {
        if (AutoPose == AutoPoses.LEFT) {

            //drive to LeftBucketScore
            scorePassive = new Path(new BezierLine(startPose.getPoint(), PoseStoragePedro.LeftBucketScore.getPoint()));
            scorePassive.setLinearHeadingInterpolation(startPose.getHeading(), PoseStoragePedro.LeftBucketScore.getHeading());
            scorePassive.setPathEndVelocityConstraint(20);

//            returnToCheckPoint = new Path(new BezierLine(lastPose.getPoint(), PoseStoragePedro.LeftCheckPoint.getPoint()));
//            returnToCheckPoint.setLinearHeadingInterpolation(lastPose.getHeading(), PoseStoragePedro.LeftCheckPoint.getHeading());
//            returnToCheckPoint.setPathEndVelocityConstraint(20); //slow down speed for arm movement

            Sample1 = new Path(new BezierLine(PoseStoragePedro.LeftBucketScore.getPoint(), PoseStoragePedro.LeftSample1.getPoint()));
            Sample1.setLinearHeadingInterpolation(PoseStoragePedro.LeftBucketScore.getHeading(), PoseStoragePedro.LeftSample1.getHeading());

            Sample2 = new Path(new BezierLine(PoseStoragePedro.LeftBucketScore.getPoint(), PoseStoragePedro.LeftSample2.getPoint()));
            Sample2.setLinearHeadingInterpolation(PoseStoragePedro.LeftBucketScore.getHeading(), PoseStoragePedro.LeftSample2.getHeading());

            Sample3 = new Path(new BezierLine(PoseStoragePedro.LeftCheckPoint.getPoint(), PoseStoragePedro.LeftSample3.getPoint()));
            Sample3.setLinearHeadingInterpolation(PoseStoragePedro.LeftCheckPoint.getHeading(), PoseStoragePedro.LeftSample3.getHeading());

            score = new Path(new BezierLine(PoseStoragePedro.LeftSample1.getPoint(), PoseStoragePedro.LeftBucketScore.getPoint()));
            score.setLinearHeadingInterpolation(PoseStoragePedro.LeftSample1.getHeading(), PoseStoragePedro.LeftBucketScore.getHeading());

            park = new Path(new BezierLine(PoseStoragePedro.LeftBucketScore.getPoint(), PoseStoragePedro.LeftPark.getPoint()));
            park.setLinearHeadingInterpolation(PoseStoragePedro.LeftBucketScore.getHeading(), PoseStoragePedro.LeftPark.getHeading());


            //TODO *************** RIGHT ****************
        } else if (AutoPose == AutoPoses.RIGHT) {

        }
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
        PoseStoragePedro.CurrentPose = bot.getPose(); //updates currentPose variable
        telemetry.addData("Selected Auto Position: ", AutoPose);
        telemetry.addData("Selected Park Position: ", AutoPose);
        telemetry.addData("Current State: ", currentState);
        telemetry.addData("X Position: ", bot.getPose().getX());
        telemetry.addData("Y Position: ", bot.getPose().getY());
        telemetry.addData("Heading Position: ", bot.getPose().getHeading());
        telemetry.addData("Message: ", message);
        telemetry.update();
    }

    //vital method to update score paths based on number of samples scored
    public void updateScoreStart(int sampleNumber){
        if(AutoPose == AutoPoses.LEFT){
            switch(sampleNumber){
                case 2:
                    score = new Path(new BezierLine(PoseStoragePedro.LeftSample2.getPoint(), PoseStoragePedro.LeftBucketScore.getPoint()));
                    score.setLinearHeadingInterpolation(PoseStoragePedro.LeftSample2.getHeading(), PoseStoragePedro.LeftBucketScore.getHeading());
                    break;

                case 3:
                    score = new Path(new BezierLine(PoseStoragePedro.LeftSample3.getPoint(), PoseStoragePedro.LeftBucketScore.getPoint()));
                    score.setLinearHeadingInterpolation(PoseStoragePedro.LeftSample3.getHeading(), PoseStoragePedro.LeftBucketScore.getHeading());
                    break;
            }
        } else if(AutoPose == AutoPoses.RIGHT){

        }
    }

        public void runOpMode () throws InterruptedException {
            bot = new Follower(hardwareMap);

            startPose = PoseStoragePedro.LeftStartPose;
            parkPose = PoseStoragePedro.LeftPark;

            driverOp = new GamepadEx(gamepad1);

            //initialize subsystems
            init_classes(driverOp);


            while (opModeInInit()) {
                runMenu();
                AutoPose = getAutoPose();
                telemetry.update();
            }

            waitForStart();

            buildPaths(AutoPose); //builds paths after we select the autoStart pose from the menu

            bot.setPose(startPose);

            // starting path & FSM
            currentState = State.SCORE_PASSIVE;
            bot.followPath(scorePassive, true);
            //TODO move arm up
            //TODO run intake (pickup) - should secure passive/loaded sample


            //TODO change start poses in all paths to match Auto Logic!!!!!!!!
            while (opModeIsActive()) {
                // FSM Auto Logic
                switch (currentState) {
                    case SCORE_PASSIVE:
                        if (!bot.isBusy()) {
                            waitSeconds(0.3);
                            //TODO run intake (drop)
                            waitSeconds(.1);
                            //TODO move arm down
                            currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                            bot.followPath(Sample1);
                            //TODO run intake (pickup)
                            break;


                        }

                    case GET_GROUND_SAMPLE:
                        if (!bot.isBusy()) {
                            waitSeconds(.3);
                            currentState = Sample_Auto.State.SCORE;
                            if(groundSamplesScored == 1){
                                updateScoreStart(2);
                            } else if (groundSamplesScored == 2){
                                updateScoreStart(3);
                            }
                            bot.followPath(score);
                            //TODO move arm up
                            break;
                        }

                    case SCORE:
                        if (!bot.isBusy()) {
                            waitSeconds(0.3);
                            //TODO run intake (drop)
                            groundSamplesScored++;

                            if (groundSamplesScored == 1) {
                                currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                bot.followPath(Sample2);
                                //TODO move arm down
                                //TODO run intake (pickup)
                                break;
                            } else if (groundSamplesScored == 2) {
                                currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                bot.followPath(Sample3);
                                //TODO move arm down
                                //TODO run intake (pickup)
                                break;
                            } else {
                                currentState = Sample_Auto.State.PARK;
                                bot.followPath(park);
                                //TODO move arm down (for parking)
                                //TODO stop intake
                                break;
                            }
                        }
                    case PARK:
                        if (!bot.isBusy()) {
                            waitSeconds(0.3);
                            //TODO move arm down (a bit for max parking)
                            currentState = Sample_Auto.State.END;
                        }
                }
                updateAuto();
            }
        }
    }

