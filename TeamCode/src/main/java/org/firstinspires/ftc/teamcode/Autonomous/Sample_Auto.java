package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Utilities.PoseStoragePedro;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

@Autonomous(name="3 Sample Auto", group = "Autos")
public class Sample_Auto extends AutoBase {
    //important variables
    Path scorePassive, Sample1, Sample2, Sample3, Score, Park, SampleDropoff, PickupSpecimen;
    PathChain scorePassiveChain;
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
        PICKUP_SPECIMEN, //right side only
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
            scorePassive = new Path(new BezierLine(startPose.getPoint(), PoseStoragePedro.SpecimenScore.getPoint()));
            scorePassive.setLinearHeadingInterpolation(startPose.getHeading(), PoseStoragePedro.SpecimenScore.getHeading());
            scorePassive.setPathEndVelocityConstraint(20);

            scorePassiveChain = bot.pathBuilder()
                    .addPath(new BezierLine(startPose.getPoint(), new Point(new Pose(startPose.getX(),startPose.getY()+3,startPose.getHeading()))))
                    .setConstantHeadingInterpolation(startPose.getHeading())
                    .addPath(new BezierLine(new Point(new Pose(startPose.getX(),startPose.getY()+3,startPose.getHeading())), PoseStoragePedro.SampleDropoff.getPoint()))
                    .setLinearHeadingInterpolation(startPose.getHeading(), PoseStoragePedro.SampleDropoff.getHeading())
                    .setPathEndVelocityConstraint(20)
                    .build();

            Sample1 = new Path(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightSample1.getPoint()));
            Sample1.setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightSample1.getHeading());

            Sample2 = new Path(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightSample2.getPoint()));
            Sample2.setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightSample2.getHeading());

            Sample3 = new Path(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightSample3.getPoint()));
            Sample3.setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightSample3.getHeading());

            SampleDropoff = new Path(new BezierLine(PoseStoragePedro.RightSample1.getPoint(), PoseStoragePedro.SampleDropoff.getPoint()));
            SampleDropoff.setLinearHeadingInterpolation(PoseStoragePedro.RightSample1.getHeading(), PoseStoragePedro.SampleDropoff.getHeading());

            PickupSpecimen = new Path(new BezierLine(PoseStoragePedro.SampleDropoff.getPoint(), PoseStoragePedro.SpecimenPickup.getPoint()));
            PickupSpecimen.setLinearHeadingInterpolation(PoseStoragePedro.SampleDropoff.getHeading(), PoseStoragePedro.SpecimenPickup.getHeading());

            Score = new Path(new BezierLine(PoseStoragePedro.SpecimenPickup.getPoint(), PoseStoragePedro.SpecimenScore.getPoint()));
            Score.setLinearHeadingInterpolation(PoseStoragePedro.SpecimenPickup.getHeading(), PoseStoragePedro.SpecimenScore.getHeading());

            Park = new Path(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightPark.getPoint()));
            Park.setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightPark.getHeading());
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
        //arm.runPassiveExtend();
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
            switch(sampleNumber){
                case 2:
                    SampleDropoff = new Path(new BezierLine(PoseStoragePedro.RightSample2.getPoint(), PoseStoragePedro.SampleDropoff.getPoint()));
                    SampleDropoff.setLinearHeadingInterpolation(PoseStoragePedro.RightSample2.getHeading(), PoseStoragePedro.SampleDropoff.getHeading());
                    break;

                case 3:
                    SampleDropoff = new Path(new BezierLine(PoseStoragePedro.RightSample3.getPoint(), PoseStoragePedro.SampleDropoff.getPoint()));
                    SampleDropoff.setLinearHeadingInterpolation(PoseStoragePedro.RightSample3.getHeading(), PoseStoragePedro.SampleDropoff.getHeading());
                    break;
            }
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

            if(AutoPose == AutoPoses.LEFT){
                bot.setPose(PoseStoragePedro.LeftStartPose);
            } else if(AutoPose == AutoPoses.RIGHT){
                bot.setPose(PoseStoragePedro.RightStartPose);
            }

            // starting path & FSM
            currentState = State.SCORE_PASSIVE;
            bot.followPath(scorePassive, true);

            //arm.setPose(Arm.Arm_Poses.SPECIMEN_PICKUP); //TODO change to BASKET2 later
            intake.pickup(); //should secure passive/loaded sample



            while (opModeIsActive()) {
                // FSM Auto Logic
                if(AutoPose == AutoPoses.LEFT) {
                    switch(currentState) {
                        case SCORE_PASSIVE:
                            if (!bot.isBusy()) {
                                waitSeconds(0.2);
                                intake.drop(); //score
                                waitSeconds(1);
                                intake.stop();
                                //arm.setPose(Arm.Arm_Poses.GROUND);
                                currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                bot.followPath(Sample1, true);
                                intake.pickup();
                                break;
                            }

                        case GET_GROUND_SAMPLE:
                            if (!bot.isBusy()) {
                                waitSeconds(.5);
                                intake.stop();
                                if (groundSamplesScored == 1) {
                                    updateScoreStart(2);
                                } else if (groundSamplesScored == 2) {
                                    updateScoreStart(3);
                                }

                                currentState = State.SCORE;
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
                                waitSeconds(.1);
                                intake.drop();
                                waitSeconds(.1);
                                intake.stop();

                                currentState = State.GET_GROUND_SAMPLE;
                                bot.followPath(Sample1, true);
                                break;

                            };
                        case GET_GROUND_SAMPLE:
                            //TODO add updateScoreStart()
                            if (!bot.isBusy()) {
                                intake.pickup();
                                waitSeconds(1);
                                intake.stop();

                                if(groundSamplesScored == 1){
                                    updateScoreStart(2);
                                } else if(groundSamplesScored == 2){
                                    updateScoreStart(3);
                                }

                                currentState = State.DROPOFF_SAMPLE;
                                bot.followPath(SampleDropoff, true);
                                break;
                            };
                        case DROPOFF_SAMPLE:
                            if (!bot.isBusy()) {
                                waitSeconds(0.3);

                                currentState = State.PICKUP_SPECIMEN;
                                bot.followPath(PickupSpecimen, true);
                                break;
                            };
                        case PICKUP_SPECIMEN:
                            if(!bot.isBusy()){
                                waitSeconds(.3);
                                currentState = State.SCORE;
                                bot.followPath(Score, true);
                                break;
                            }
                        case SCORE: //TODO------------------
                            if (!bot.isBusy()) {
                                waitSeconds(.3);
                                groundSamplesScored++;
                                if(groundSamplesScored == 1){
                                    //sample2
                                    currentState = State.GET_GROUND_SAMPLE;
                                    bot.followPath(Sample2);
                                    break;
                                } else if(groundSamplesScored == 2){
                                    //sample3
                                    currentState = State.GET_GROUND_SAMPLE;
                                    bot.followPath(Sample3);
                                    break;
                                } else {
                                    //park
                                    currentState = State.PARK;
                                    bot.followPath(Park);
                                    break;
                                }

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