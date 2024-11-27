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
import java.util.concurrent.TimeUnit;

@Autonomous(name="3 Sample Auto", group = "Autos")
public class Sample_Auto extends AutoBase {
    //important variables
    Path scorePassive, Sample1, Sample2, Sample3, Score, Park, SampleDropoff;
    Pose startPose;
    Pose parkPose;
    Follower bot;
    AutoPoses AutoPose = getAutoPose();
    GamepadEx driverOp;
    String message = "";


    //Finite State Machine (FSM) variables
    public enum State {
        SCORE_PASSIVE, //path to buckets core (left)/sample dropoff (right) from startPose
        GET_GROUND_SAMPLE,
        DROPOFF_SAMPLE, //right side only
        SCORE,
        PARK,
        END
    }

    public State currentState;
    public int groundSamplesScored = 0;


    //add all paths/auto objectives here
    public void buildPaths(AutoPoses AutoPose) {
        if (AutoPose == AutoPoses.LEFT) {

            scorePassive = new Path(new BezierLine(startPose.getPoint(), PoseStoragePedro.LeftBucketScore.getPoint()));
            scorePassive.setLinearHeadingInterpolation(startPose.getHeading(), PoseStoragePedro.LeftBucketScore.getHeading());
            scorePassive.setPathEndVelocityConstraint(20);

            Sample1 = new Path(new BezierLine(PoseStoragePedro.LeftBucketScore.getPoint(), PoseStoragePedro.LeftSample1.getPoint()));
            Sample1.setLinearHeadingInterpolation(PoseStoragePedro.LeftBucketScore.getHeading(), PoseStoragePedro.LeftSample1.getHeading());

            Sample2 = new Path(new BezierLine(PoseStoragePedro.LeftBucketScore.getPoint(), PoseStoragePedro.LeftSample2.getPoint()));
            Sample2.setLinearHeadingInterpolation(PoseStoragePedro.LeftBucketScore.getHeading(), PoseStoragePedro.LeftSample2.getHeading());

            Sample3 = new Path(new BezierLine(PoseStoragePedro.LeftCheckPoint.getPoint(), PoseStoragePedro.LeftSample3.getPoint()));
            Sample3.setLinearHeadingInterpolation(PoseStoragePedro.LeftCheckPoint.getHeading(), PoseStoragePedro.LeftSample3.getHeading());

            Score = new Path(new BezierLine(PoseStoragePedro.LeftSample1.getPoint(), PoseStoragePedro.LeftBucketScore.getPoint()));
            Score.setLinearHeadingInterpolation(PoseStoragePedro.LeftSample1.getHeading(), PoseStoragePedro.LeftBucketScore.getHeading());

            Park = new Path(new BezierLine(PoseStoragePedro.LeftBucketScore.getPoint(), PoseStoragePedro.LeftPark.getPoint()));
            Park.setLinearHeadingInterpolation(PoseStoragePedro.LeftBucketScore.getHeading(), PoseStoragePedro.LeftPark.getHeading());



        } else if (AutoPose == AutoPoses.RIGHT) {
            scorePassive = new Path(new BezierLine(startPose.getPoint(), PoseStoragePedro.SampleDropoff.getPoint()));
            scorePassive.setLinearHeadingInterpolation(startPose.getHeading(), PoseStoragePedro.SampleDropoff.getHeading());
            scorePassive.setPathEndVelocityConstraint(20);

            Sample1 = new Path(new BezierLine(PoseStoragePedro.SampleDropoff.getPoint(), PoseStoragePedro.RightSample1.getPoint()));
            Sample1.setLinearHeadingInterpolation(PoseStoragePedro.SampleDropoff.getHeading(), PoseStoragePedro.RightSample1.getHeading());
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
        subsystemsUpdate();
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
                    Score = new Path(new BezierLine(PoseStoragePedro.LeftSample2.getPoint(), PoseStoragePedro.LeftBucketScore.getPoint()));
                    Score.setLinearHeadingInterpolation(PoseStoragePedro.LeftSample2.getHeading(), PoseStoragePedro.LeftBucketScore.getHeading());
                    break;

                case 3:
                    Score = new Path(new BezierLine(PoseStoragePedro.LeftSample3.getPoint(), PoseStoragePedro.LeftBucketScore.getPoint()));
                    Score.setLinearHeadingInterpolation(PoseStoragePedro.LeftSample3.getHeading(), PoseStoragePedro.LeftBucketScore.getHeading());
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
            intake.pickup(); //should secure passive/loaded sample


            //TODO change start poses in all paths to match Auto Logic!!!!!!!!
            while (opModeIsActive()) {
                // FSM Auto Logic
                if(AutoPose == AutoPoses.LEFT) {
                    switch(currentState) {
                        case SCORE_PASSIVE:
                            if (!bot.isBusy()) {
                                intake.stop();
                                waitSeconds(0.3);
                                intake.drop();
                                waitSeconds(1);
                                intake.stop();
                                //TODO move arm down
                                currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                bot.followPath(Sample1, true);
                                intake.pickup();
                                break;
                            }

                        case GET_GROUND_SAMPLE:
                            if (!bot.isBusy()) {
                                waitSeconds(.5);
                                intake.stop();
                                currentState = Sample_Auto.State.SCORE;
                                if (groundSamplesScored == 1) {
                                    updateScoreStart(2);
                                } else if (groundSamplesScored == 2) {
                                    updateScoreStart(3);
                                }
                                bot.followPath(Score, true);
                                //TODO move arm up
                                break;
                            }

                        case SCORE:
                            if (!bot.isBusy()) {
                                waitSeconds(0.3);
                                intake.drop();
                                waitSeconds(1);
                                groundSamplesScored++;

                                if (groundSamplesScored == 1) {
                                    currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                    bot.followPath(Sample2, true);
                                    intake.pickup();
                                    //TODO move arm down
                                    //TODO run intake (pickup)
                                    break;
                                } else if (groundSamplesScored == 2) {
                                    currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                    bot.followPath(Sample3, true);
                                    intake.pickup();
                                    //TODO move arm down
                                    //TODO run intake (pickup)
                                    break;
                                } else {
                                    currentState = Sample_Auto.State.PARK;
                                    bot.followPath(Park, true);
                                    intake.pickup();
                                    //TODO move arm down (for parking)
                                    //TODO stop intake
                                    break;
                                }
                            }
                        case PARK:
                            if (!bot.isBusy()) {
                                intake.stop();
                                waitSeconds(0.3);
                                //TODO move arm down (a bit for max parking)
                                currentState = Sample_Auto.State.END;
                            }
                    }
                } else if(AutoPose == AutoPoses.RIGHT){
                    switch(currentState){
                        case SCORE_PASSIVE:
                            if (!bot.isBusy()) {
                                intake.stop();
                                waitSeconds(1);
                                intake.drop();
                                waitSeconds(1);
                                intake.stop();

                                currentState = State.GET_GROUND_SAMPLE;
                                bot.followPath(Sample1);
                                break;

                            };
                        case GET_GROUND_SAMPLE:
                            if (!bot.isBusy()) {
                                intake.pickup();
                                waitSeconds(1);
                                intake.stop();

                                currentState = State.END;
                                break;
                            };
                        case DROPOFF_SAMPLE:
                            if (!bot.isBusy()) {
                                waitSeconds(0.3);

                                currentState = State.END;
                                break;
                            };
                        case SCORE:
                            if (!bot.isBusy()) {
                                waitSeconds(0.3);

                                currentState = State.END;
                                break;
                            }
                        case PARK:
                            if (!bot.isBusy()) {
                                waitSeconds(0.3);
                                //TODO move arm down (a bit for max parking)
                                currentState = Sample_Auto.State.END;
                                break;
                            }
                    }
                }
                updateAuto();
            }
        }
    }

