package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.util.Drawing;

import java.util.concurrent.TimeUnit;

@Autonomous(name="4 Cycle Auto", group = "Autos")
public class Four_Cycle_Auto extends AutoBase {
    //important variables
    Path scorePassive, SampleDropoff, PickupSpecimen;
    PathChain scorePassiveChain, Sample1, Sample2, Sample3, Sample3Back, Sample2Back, specimenPickup, specimenAlignment, Score, Park;
    Pose startPose;
    Pose parkPose;
    Follower bot;
    AutoPoses AutoPose = getAutoPose();
    GamepadEx driverOp;
    String message = "";
    ElapsedTime pathTimer;
    ElapsedTime opModeTimer;
    double endTime = 0;
    boolean back2PathDone = false;
    boolean back3PathDone = false;
    boolean alignPathDone = false;
    double backAmount = 6; //4.75


    //Finite State Machine (FSM) variables
    public enum State {
        SCORE_PASSIVE, //path to buckets (left)/sample dropoff (right) from startPose
        GET_GROUND_SAMPLE,
        MOVE_SAMPLES, //right side only
        PICKUP_SPECIMEN, //right side only
        SCORE,
        PARK,
        END
    }

    public State currentState;
    public int groundSamplesScored = 0;
    public int specimenScored = 0;
    public int groundSamplesRetrieved = 0;


    //add all paths/auto objectives here
    public void buildPaths(AutoPoses AutoPose) {
        if (AutoPose == AutoPoses.LEFT) {
            scorePassive = new Path(new BezierLine(startPose.getPoint(), PoseStorage.LeftBucketScore.getPoint()));
            scorePassive.setLinearHeadingInterpolation(startPose.getHeading(), PoseStorage.LeftBucketScore.getHeading());
            scorePassive.setPathEndVelocityConstraint(20);

            Sample1 = bot.pathBuilder() //drives to control pose first then leftSample1 to scoop sample
                    .addPath(new BezierLine(PoseStorage.LeftBucketScore.getPoint(), PoseStorage.LeftSampleControlPose.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.LeftBucketScore.getHeading(), PoseStorage.LeftSampleControlPose.getHeading())
                    .setPathEndVelocityConstraint(10)
                    .addPath(new BezierLine(PoseStorage.LeftSampleControlPose.getPoint(), PoseStorage.LeftSample1.getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.LeftSampleControlPose.getHeading())
                    .build();


            Sample2 = bot.pathBuilder() //drives to sample1 pose first then sample2 pose to scoop sample
                    .addPath(new BezierLine(PoseStorage.LeftBucketScore.getPoint(), PoseStorage.LeftSample1.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.LeftBucketScore.getHeading(), PoseStorage.LeftSample1.getHeading())
                    .setPathEndVelocityConstraint(10)
                    .addPath(new BezierLine(PoseStorage.LeftSample1.getPoint(), PoseStorage.LeftSample2.getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.LeftSample1.getHeading())
                    .build();

            Sample2Back = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStorage.LeftSample2.getPoint(), PoseStorage.LeftSample3Back.getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.LeftSample2.getHeading())
                    .build();

            Sample3 = bot.pathBuilder() //drives to sample2 pose first then sample3 to scoop sample
                    .addPath(new BezierLine(PoseStorage.LeftBucketScore.getPoint(), PoseStorage.LeftSample2.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.LeftBucketScore.getHeading(), PoseStorage.LeftSample2.getHeading())
                    .setPathEndVelocityConstraint(10)
                    .addPath(new BezierLine(PoseStorage.LeftSample2.getPoint(), PoseStorage.LeftSample3.getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.LeftSample2.getHeading())
                    .build();

            Sample3Back = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStorage.LeftSample3.getPoint(), new Pose(PoseStorage.LeftSample3.getX()-4,PoseStorage.LeftSample3.getY(),PoseStorage.LeftSample3.getHeading()).getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.LeftSample3.getHeading())
                    .addPath(new BezierLine(PoseStorage.LeftSample3.getPoint(), PoseStorage.LeftSample3Back.getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.LeftSample3.getHeading())
                    .build();


            Score = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStorage.LeftSample1.getPoint(), PoseStorage.LeftBucketScore.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.LeftSample1.getHeading(), PoseStorage.LeftBucketScore.getHeading())
                    .build();

            Park = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStorage.LeftBucketScore.getPoint(), PoseStorage.LeftPark.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.LeftBucketScore.getHeading(), PoseStorage.LeftPark.getHeading())
                    .build();


        } else if (AutoPose == AutoPoses.RIGHT) {

            scorePassiveChain = bot.pathBuilder()
                    .addPath(new BezierLine(startPose.getPoint(),PoseStorage.SpecimenScore.getPoint()))
                    .setLinearHeadingInterpolation(startPose.getHeading(),PoseStorage.SpecimenScore.getHeading())
                    .build();

            Sample1 = bot.pathBuilder()
                    //move back to release specimen
                    .addPath(new BezierLine(PoseStorage.SpecimenScore.getPoint(), PoseStorage.SpecimenScoreBackup.getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.SpecimenScore.getHeading())

                    //go to sample1Start pose
                    .addPath(new BezierLine(PoseStorage.SpecimenScoreBackup.getPoint(), PoseStorage.RightSample1Start.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.SpecimenScoreBackup.getHeading(),PoseStorage.RightSample1Start.getHeading())

                    .addPath(new BezierCurve(PoseStorage.RightSample1Start.getPoint(),PoseStorage.RightSample1Control1,PoseStorage.RightSample1Control2, PoseStorage.RightSample1.getPoint()))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .addPath(new BezierLine(PoseStorage.RightSample1.getPoint(),PoseStorage.RightSample1Push.getPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Sample2 = bot.pathBuilder()
                    .addPath(new BezierCurve(PoseStorage.RightSample1Push.getPoint(),PoseStorage.RightSample2Control1,PoseStorage.RightSample2.getPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(new BezierLine(PoseStorage.RightSample2.getPoint(),PoseStorage.RightSample2Push.getPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Sample3 = bot.pathBuilder()
                    .addPath(new BezierCurve(PoseStorage.RightSample2Push.getPoint(),PoseStorage.RightSample3Control1,PoseStorage.RightSample3.getPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(new BezierLine(PoseStorage.RightSample3.getPoint(),PoseStorage.RightSample3Push.getPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            specimenPickup = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStorage.RightSample3Push.getPoint(),PoseStorage.SpecimenPickup.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.RightSample3Push.getHeading(), PoseStorage.SpecimenPickup.getHeading())
                    .setPathEndTimeoutConstraint(1)
                    .build();

            Score = bot.pathBuilder()
                    .addPath(new BezierLine(new Pose(PoseStorage.SpecimenPickup.getX(),PoseStorage.SpecimenPickup.getY()-backAmount,PoseStorage.SpecimenPickup.getHeading()).getPoint(),PoseStorage.SpecimenScore2.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.SpecimenPickup.getHeading(), PoseStorage.SpecimenScore2.getHeading())
                    .build();

            Park = bot.pathBuilder()
                    //backup
                    .addPath(new BezierLine(PoseStorage.SpecimenScore.getPoint(),PoseStorage.RightParkBack.getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.SpecimenScore.getHeading())

                    //go to park
                    .addPath(new BezierLine(PoseStorage.RightParkBack.getPoint(),PoseStorage.RightPark.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.RightParkBack.getHeading(), PoseStorage.RightPark.getHeading())
                    .build();

             }
    }

    //native wait method - seconds
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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    //various PID updating & Telemetry Data bundled into one method
    public void updateAuto(){
        bot.update(); //controls Pedro-Pathing logic
        subsystemsUpdate();
        //arm.runPassiveExtend();
        PoseStorage.CurrentPose = bot.getPose(); //updates currentPose variable
        mainTelemetry.addLine("---AUTO DATA---");
        mainTelemetry.addData("Auto Ran: ", PoseStorage.ranAuto);

        if(endTime != 0){
            mainTelemetry.addData("OpMode Timer: ", endTime);
        } else {
            mainTelemetry.addData("OpMode Timer: ", opModeTimer.seconds());
        }

        mainTelemetry.addData("Selected Auto Position: ", AutoPose);
        mainTelemetry.addData("Selected Park Position: ", AutoPose);
        mainTelemetry.addData("Current State: ", currentState);
        mainTelemetry.addData("X Position: ", bot.getPose().getX());
        mainTelemetry.addData("Y Position: ", bot.getPose().getY());
        mainTelemetry.addData("Heading Position: ", bot.getPose().getHeading());
        mainTelemetry.addData("Total Heading: ", bot.getTotalHeading());
        mainTelemetry.addData("Message: ", message);
        mainTelemetry.addData("Path Timer: ", pathTimer.time(TimeUnit.SECONDS));
        if(AutoPose == AutoPoses.LEFT){
            mainTelemetry.addData("Samples Scored: ", groundSamplesScored);
        } else {
            mainTelemetry.addData("Specimens Scored: ", groundSamplesScored);
        }
        getSubsystemTelemetry(mainTelemetry);

        //Draw Bot on Dashboard
        Drawing.drawPoseHistory(bot.getDashboardPoseTracker(), "#be87e8"); //Light Purple
        Drawing.drawRobot(bot.getPose(), "#8F2CDB"); //Main Purple
        Drawing.sendPacket();

        mainTelemetry.update();
    }

    public void buildStrafePath(double length){
        Pose specimenPickup = PoseStorage.SpecimenPickup;
                if(length != 0){
                    specimenAlignment = bot.pathBuilder()
                            //strafe slowly
                            .addPath(new BezierLine(specimenPickup.getPoint(), new Pose(specimenPickup.getX() + length,specimenPickup.getY(),specimenPickup.getHeading()).getPoint()))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .addParametricCallback(0,()->{
                                bot.setMaxPower(0.5); //slow speed
                            })
                            .setPathEndTimeoutConstraint(1.5)


                            //drive into specimen
                            .addPath(new BezierLine(new Pose(specimenPickup.getX() + length,specimenPickup.getY(),specimenPickup.getHeading()).getPoint(), new Pose(specimenPickup.getX() + length,specimenPickup.getY() - 5,specimenPickup.getHeading()).getPoint()))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .addParametricCallback(0,()->{
                                bot.setMaxPower(1);
                            })
                            .setPathEndTimeoutConstraint(1.5)


                            //drive back to pickup pose
                            .addPath(new BezierLine(new Pose(specimenPickup.getX() + length,specimenPickup.getY() - 3,specimenPickup.getHeading()).getPoint(), specimenPickup.getPoint()))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .setPathEndTimeoutConstraint(1.5)

                            .build();
                } else {
                    specimenAlignment = bot.pathBuilder()
                            //drive into specimen
                            .addPath(new BezierLine(specimenPickup.getPoint(), new Pose(specimenPickup.getX(),specimenPickup.getY()-backAmount,specimenPickup.getHeading()).getPoint()))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .setPathEndTimeoutConstraint(1.5)
                            //.setPathEndTValueConstraint(1.5)

                            .build();
                }
    }

    //vital method to update score paths based on number of samples scored
    public void updateScoreStart(int sampleNumber){
        if(AutoPose == AutoPoses.LEFT){
            switch(sampleNumber){
                case 2:

                case 3:
                    Score = bot.pathBuilder()
                            .addPath(new BezierLine(PoseStorage.LeftSample3Back.getPoint(), PoseStorage.LeftBucketScore.getPoint()))
                            .setLinearHeadingInterpolation(PoseStorage.LeftSample3Back.getHeading(), PoseStorage.LeftBucketScore.getHeading())
                            .build();
                    break;

            }
        } else if(AutoPose == AutoPoses.RIGHT){
            switch (sampleNumber){
                case 1:
                    specimenPickup = bot.pathBuilder()
                            .addPath(new BezierLine(PoseStorage.SpecimenScore2.getPoint(), PoseStorage.SpecimenPickup.getPoint()))
                            .setLinearHeadingInterpolation(PoseStorage.SpecimenScore2.getHeading(),PoseStorage.SpecimenPickup.getHeading())
                            .build();

                    Score = bot.pathBuilder()
                            .addPath(new BezierLine(PoseStorage.SpecimenPickup.getPoint(), PoseStorage.SpecimenScore2.getPoint()))
                            .setLinearHeadingInterpolation(PoseStorage.SpecimenPickup.getHeading(), PoseStorage.SpecimenScore2.getHeading())
                            .build();

                    break;
                case 2:
                    //logic
                    specimenPickup = bot.pathBuilder()
                            .addPath(new BezierLine(PoseStorage.SpecimenScore3.getPoint(), PoseStorage.SpecimenPickup.getPoint()))
                            .setLinearHeadingInterpolation(PoseStorage.SpecimenScore3.getHeading(),PoseStorage.SpecimenPickup.getHeading())
                            .build();

                    Score = bot.pathBuilder()
                            .addPath(new BezierLine(PoseStorage.SpecimenPickup.getPoint(), PoseStorage.SpecimenScore3.getPoint()))
                            .setLinearHeadingInterpolation(PoseStorage.SpecimenPickup.getHeading(), PoseStorage.SpecimenScore3.getHeading())
                            .build();
                    break;
                case 3:
                    //logic
                    specimenPickup = bot.pathBuilder()
                            .addPath(new BezierLine(PoseStorage.SpecimenScore4.getPoint(), PoseStorage.SpecimenPickup.getPoint()))
                            .setLinearHeadingInterpolation(PoseStorage.SpecimenScore4.getHeading(),PoseStorage.SpecimenPickup.getHeading())
                            .build();

                    Score = bot.pathBuilder()
                            .addPath(new BezierLine(PoseStorage.SpecimenPickup.getPoint(), PoseStorage.SpecimenScore4.getPoint()))
                            .setLinearHeadingInterpolation(PoseStorage.SpecimenPickup.getHeading(), PoseStorage.SpecimenScore4.getHeading())
                            .build();
                    break;
            }

        }
    }

    //auto loop
        public void runOpMode () throws InterruptedException {
            bot = new Follower(hardwareMap);
            PoseStorage.ranAuto = false;
            startPose = PoseStorage.LeftStartPose;
            parkPose = PoseStorage.LeftPark;
            driverOp = new GamepadEx(gamepad1);
            pathTimer = new ElapsedTime();
            opModeTimer = new ElapsedTime();

            //initialize subsystems
            init_classes(driverOp);

            //init loop
            while (opModeInInit()) {
                runMenu(mainTelemetry);
                AutoPose = getAutoPose();
                PoseStorage.allianceSide = getAutoPose();
                mainTelemetry.update();
            }

            waitForStart();

            buildPaths(AutoPose); //builds paths after we select the autoStart pose from the menu

            if(AutoPose == AutoPoses.LEFT){
                bot.setPose(PoseStorage.LeftStartPose);
                // starting path & FSM
                //TODO bot.setSamplePID(true);
                currentState = State.SCORE_PASSIVE;
                intake.pickup(); //should secure passive/loaded sample
                arm.setTarget(5700,4000); //5600,3250
                //waitSeconds(0.01);
                bot.setMaxPower(1); //.9
                bot.followPath(scorePassive, true);
                pathTimer.reset();
            } else if(AutoPose == AutoPoses.RIGHT){
                bot.setPose(PoseStorage.RightStartPose); //different from bot.setPose()
                // starting path & FSM
                currentState = State.SCORE_PASSIVE;
                bot.setMaxPower(.9); //.8
                lift.setTarget(1400);
                bot.followPath(scorePassiveChain, true);
                pathTimer.reset();
            }


            //Draw initial bot pose on field (FTC Dashboard)
            Drawing.drawRobot(bot.getPose(), "#4CAF50");
            Drawing.sendPacket();


            while (opModeIsActive()) {
                // FSM Auto Logic
                if(AutoPose == AutoPoses.LEFT) {
                    switch(currentState) {
                        case SCORE_PASSIVE:
                            if (!bot.isBusy()) {
                                waitMilliSeconds(500);
                                //waitSeconds(.05);
                                intake.drop(); //score
                                waitMilliSeconds(600);
                                intake.stop();
                                arm.setTarget(0,0);
                                currentState = Four_Cycle_Auto.State.GET_GROUND_SAMPLE;
                                bot.followPath(Sample1, true);
                                intake.pickup();
                                pathTimer.reset();
                                break;
                            }

                        case GET_GROUND_SAMPLE:
                            if (!bot.isBusy()) {
                                waitMilliSeconds(500);
                                if(!back2PathDone && groundSamplesScored == 1){
                                   // waitSeconds(.01);
                                    back2PathDone = true;
                                    bot.followPath(Sample2Back);
                                }else if(!back3PathDone && groundSamplesScored == 2){
                                    //waitSeconds(.01); //.1
                                    intake.pickup(); //TODO
                                    back3PathDone = true;
                                    bot.setMaxPower(0.6);
                                    bot.followPath(Sample3Back);
                                } else {
                                    //waitSeconds(.05); //.1
                                    //intake.stop();
                                    bot.setMaxPower(1);
                                    if (groundSamplesScored == 1) {
                                        updateScoreStart(2);
                                    } else if (groundSamplesScored == 2) {
                                        updateScoreStart(3);
                                    }

                                    arm.setTarget(5700, 4000);
                                    //waitSeconds(.01); //.1
                                    currentState = State.SCORE;
                                    bot.setMaxPower(1); //.9
                                    bot.followPath(Score, true);
                                    pathTimer.reset();
                                    break;
                                }
                            }

                        case SCORE:
                            if (!bot.isBusy()) {
                                waitMilliSeconds(500);
                                intake.drop();
                                waitMilliSeconds(600);
                                groundSamplesScored++;

                                if(groundSamplesScored < cycleCount) {
                                    if (groundSamplesScored == 1) {
                                        currentState = Four_Cycle_Auto.State.GET_GROUND_SAMPLE;
                                        bot.setMaxPower(1); //.9
                                        bot.followPath(Sample2, true);
                                        arm.setTarget(0, 0);
                                        intake.pickup();
                                        pathTimer.reset();
                                        break;
                                    } else if (groundSamplesScored == 2) {
                                        currentState = Four_Cycle_Auto.State.GET_GROUND_SAMPLE;
                                        bot.setMaxPower(1); //.9
                                        bot.followPath(Sample3, true);
                                        arm.setTarget(0, 0);
                                        intake.pickup();
                                        pathTimer.reset();
                                        break;
                                    }
                                } else {
                                    currentState = Four_Cycle_Auto.State.PARK;
                                    bot.setMaxPower(1); //.9
                                    bot.followPath(Park, true);
                                    arm.setTarget(2800, 1000);
                                    intake.stop();
                                    pathTimer.reset();
                                    break;
                                }
                            }
                        case PARK:
                            if (!bot.isBusy()) {
                                PoseStorage.ranAuto = true;
                                endTime = opModeTimer.seconds();
                                arm.setTarget(2700,1000);
                                currentState = Four_Cycle_Auto.State.END;
                            }
                    }
                } else if(AutoPose == AutoPoses.RIGHT){
                    switch(currentState) {
                        case SCORE_PASSIVE:
                            if(!bot.isBusy()) {
                                waitMilliSeconds(50);
                                lift.setTarget(900);
                                waitMilliSeconds(800);
                                lift.setTarget(0);
                                currentState = State.MOVE_SAMPLES;
                                bot.setMaxPower(0.9); //.8
                                bot.followPath(Sample1);
                                break;
                            }

                        case MOVE_SAMPLES:
                            if(!bot.isBusy()){
                                groundSamplesRetrieved++;

                                //pickup rest of specimen
                                if(groundSamplesRetrieved == 1){
                                    //pickup 2
                                    bot.followPath(Sample2);
                                } else if(groundSamplesRetrieved == 2){
                                    //pickup 3
                                    bot.followPath(Sample3);
                                } else {

                                    currentState = State.PICKUP_SPECIMEN;
                                    bot.followPath(specimenPickup);
                                    updateScoreStart(1); //update for later
                                    break;
                                }
                            }

                        case PICKUP_SPECIMEN:
                            if(!bot.isBusy()) {
                                if(!alignPathDone){
                                    alignPathDone = true;
                                    //waitMilliSeconds(250); //time to stabilize vision reading
                                    buildStrafePath(0);//buildStrafePath(vision.getAlignment());
                                    waitMilliSeconds(100);
                                    bot.setMaxPower(0.9);
                                    bot.followPath(specimenAlignment);
                                } else {
                                    waitMilliSeconds(250);
                                    lift.setTarget(1500);
                                    waitMilliSeconds(500);
                                    updateScoreStart(specimenScored + 1);
                                    bot.setMaxPower(.9); //.75
                                    currentState = State.SCORE;
                                    bot.followPath(Score);
                                    break;
                                    }

                            }

                        case SCORE:
                            if(!bot.isBusy()) {
                                specimenScored++;
                                waitMilliSeconds(50);
                                lift.setTarget(900);
                                waitMilliSeconds(800);
                                if(specimenScored < cycleCount + 1){ //ability to score 4-5
                                    currentState = State.PICKUP_SPECIMEN;
                                    alignPathDone = false;
                                    lift.setTarget(0);
                                    bot.setMaxPower(0.8); //.8
                                    bot.followPath(specimenPickup);
                                } else {
                                    currentState = State.PARK;
                                    lift.setTarget(0);
                                    bot.followPath(Park);
                                }
                                break;
                            }

                        case PARK:
                            if(!bot.isBusy()) {
                                //logic
                                currentState = State.END;
                                endTime = opModeTimer.seconds();
                                break;
                            }
                    }
                }
                updateAuto();
            }
        }
    }