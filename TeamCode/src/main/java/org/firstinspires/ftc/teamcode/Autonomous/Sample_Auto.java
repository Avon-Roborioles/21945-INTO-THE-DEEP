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
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.util.Drawing;

import java.util.concurrent.TimeUnit;

@Autonomous(name="3 Sample Auto", group = "Autos")
public class Sample_Auto extends AutoBase {
    //important variables
    Path scorePassive, Score, Park, SampleDropoff, PickupSpecimen;
    PathChain scorePassiveChain, Sample1, Sample2, Sample3, Sample3Back, Sample2Back;
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


    //Finite State Machine (FSM) variables
    public enum State {
        SCORE_PASSIVE, //path to buckets (left)/sample dropoff (right) from startPose
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
                    .addPath(new BezierLine(PoseStorage.LeftSample3.getPoint(), new Pose(PoseStorage.LeftSample3.getX()-3,PoseStorage.LeftSample3.getY(),PoseStorage.LeftSample3.getHeading()).getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.LeftSample3.getHeading())
                    .addPath(new BezierLine(PoseStorage.LeftSample3.getPoint(), PoseStorage.LeftSample3Back.getPoint()))
                    .setConstantHeadingInterpolation(PoseStorage.LeftSample3.getHeading())
                    .build();


            Score = new Path(new BezierLine(PoseStorage.LeftSample1.getPoint(), PoseStorage.LeftBucketScore.getPoint()));
            Score.setLinearHeadingInterpolation(PoseStorage.LeftSample1.getHeading(), PoseStorage.LeftBucketScore.getHeading());

            Park = new Path(new BezierLine(PoseStorage.LeftBucketScore.getPoint(), PoseStorage.LeftPark.getPoint()));
            Park.setLinearHeadingInterpolation(PoseStorage.LeftBucketScore.getHeading(), PoseStorage.LeftPark.getHeading());



        } else if (AutoPose == AutoPoses.RIGHT) {
            scorePassive = new Path(new BezierLine(startPose.getPoint(), PoseStorage.SpecimenScore.getPoint()));
            scorePassive.setLinearHeadingInterpolation(startPose.getHeading(), PoseStorage.SpecimenScore.getHeading());
            scorePassive.setPathEndVelocityConstraint(20);
//            scorePassive = new Path(new BezierLine(startPose.getPoint(), PoseStorage.RightPark.getPoint()));
//            scorePassive.setLinearHeadingInterpolation(startPose.getHeading(), PoseStorage.RightPark.getHeading());
//            //scorePassive.setPathEndVelocityConstraint(20);

            scorePassiveChain = bot.pathBuilder()
                    .addPath(new BezierLine(startPose.getPoint(), new Point(new Pose(startPose.getX(),startPose.getY()+3,startPose.getHeading()))))
                    .setConstantHeadingInterpolation(startPose.getHeading())
                    .addPath(new BezierLine(new Point(new Pose(startPose.getX(),startPose.getY()+3,startPose.getHeading())), PoseStorage.SampleDropoff.getPoint()))
                    .setLinearHeadingInterpolation(startPose.getHeading(), PoseStorage.SampleDropoff.getHeading())
                    .setPathEndVelocityConstraint(20)
                    .build();

            //Sample1 = new Path(new BezierLine(PoseStorage.SpecimenScore.getPoint(), PoseStorage.RightSample1.getPoint()));
            //Sample1.setLinearHeadingInterpolation(PoseStorage.SpecimenScore.getHeading(), PoseStorage.RightSample1.getHeading());
            Sample1 = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStorage.SpecimenScore.getPoint(), PoseStorage.RightSample1.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.SpecimenScore.getHeading(), PoseStorage.RightSample1.getHeading())
                    .build();

            //Sample2 = new Path(new BezierLine(PoseStorage.SpecimenScore.getPoint(), PoseStorage.RightSample2.getPoint()));
            //Sample2.setLinearHeadingInterpolation(PoseStorage.SpecimenScore.getHeading(), PoseStorage.RightSample2.getHeading());
            Sample2 = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStorage.SpecimenScore.getPoint(), PoseStorage.RightSample2.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.SpecimenScore.getHeading(), PoseStorage.RightSample2.getHeading())
                    .build();

            Sample3 = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStorage.SpecimenScore.getPoint(), PoseStorage.RightSample3.getPoint()))
                    .setLinearHeadingInterpolation(PoseStorage.SpecimenScore.getHeading(), PoseStorage.RightSample3.getHeading())
                    .build();



            SampleDropoff = new Path(new BezierLine(PoseStorage.RightSample1.getPoint(), PoseStorage.SampleDropoff.getPoint()));
            SampleDropoff.setLinearHeadingInterpolation(PoseStorage.RightSample1.getHeading(), PoseStorage.SampleDropoff.getHeading());

            PickupSpecimen = new Path(new BezierLine(PoseStorage.SampleDropoff.getPoint(), PoseStorage.SpecimenPickup.getPoint()));
            PickupSpecimen.setLinearHeadingInterpolation(PoseStorage.SampleDropoff.getHeading(), PoseStorage.SpecimenPickup.getHeading());

            Score = new Path(new BezierLine(PoseStorage.SpecimenPickup.getPoint(), PoseStorage.SpecimenScore.getPoint()));
            Score.setLinearHeadingInterpolation(PoseStorage.SpecimenPickup.getHeading(), PoseStorage.SpecimenScore.getHeading());

            Park = new Path(new BezierLine(PoseStorage.SpecimenScore.getPoint(), PoseStorage.RightPark.getPoint()));
            Park.setLinearHeadingInterpolation(PoseStorage.SpecimenScore.getHeading(), PoseStorage.RightPark.getHeading());
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

    //vital method to update score paths based on number of samples scored
    public void updateScoreStart(int sampleNumber){
        if(AutoPose == AutoPoses.LEFT){
            switch(sampleNumber){
                case 2:
                    Score = new Path(new BezierLine(PoseStorage.LeftSample3Back.getPoint(), PoseStorage.LeftBucketScore.getPoint()));
                    Score.setLinearHeadingInterpolation(PoseStorage.LeftSample3Back.getHeading(), PoseStorage.LeftBucketScore.getHeading());
                    break;

                case 3:
                    Score = new Path(new BezierLine(PoseStorage.LeftSample3Back.getPoint(), PoseStorage.LeftBucketScore.getPoint()));
                    Score.setLinearHeadingInterpolation(PoseStorage.LeftSample3Back.getHeading(), PoseStorage.LeftBucketScore.getHeading());
                    break;
            }
        } else if(AutoPose == AutoPoses.RIGHT){
            switch(sampleNumber){
                case 2:
                    SampleDropoff = new Path(new BezierLine(PoseStorage.RightSample2.getPoint(), PoseStorage.SampleDropoff.getPoint()));
                    SampleDropoff.setLinearHeadingInterpolation(PoseStorage.RightSample2.getHeading(), PoseStorage.SampleDropoff.getHeading());
                    break;

                case 3:
                    SampleDropoff = new Path(new BezierLine(PoseStorage.RightSample3.getPoint(), PoseStorage.SampleDropoff.getPoint()));
                    SampleDropoff.setLinearHeadingInterpolation(PoseStorage.RightSample3.getHeading(), PoseStorage.SampleDropoff.getHeading());
                    break;
            }
        }
    }


    //auto loop
        public void runOpMode () throws InterruptedException {
            bot = new Follower(hardwareMap);

            PoseStorage.ranAuto = false; //TODO ??? false before

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
                currentState = State.SCORE_PASSIVE;
                intake.pickup(); //should secure passive/loaded sample
                arm.setTarget(5650,4000); //5600,3250
                //waitSeconds(0.01);
                bot.setMaxPower(1); //.9
                bot.followPath(scorePassive, true);
                pathTimer.reset();
            } else if(AutoPose == AutoPoses.RIGHT){
                bot.setPose(PoseStorage.RightStartPose); //different from bot.setPose()
                // starting path & FSM
                currentState = State.SCORE_PASSIVE;
                //intake.pickup(); //should secure passive/loaded sample
                //arm.setTarget(5550,4000); //5600,3250
                //waitSeconds(0.01);
                bot.setMaxPower(1); //.9
                bot.followPath(scorePassive, true);
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
                                waitMilliSeconds(500);
                                intake.stop();
                                arm.setTarget(0,0);
                                currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                bot.followPath(Sample1, true);
                                intake.pickup();
                                pathTimer.reset();
                                break;
                            }

                        case GET_GROUND_SAMPLE:
                            if (!bot.isBusy()) {
                                if(!back2PathDone && groundSamplesScored == 1){
                                   // waitSeconds(.01);
                                    back2PathDone = true;
                                    bot.followPath(Sample2Back);
                                }else if(!back3PathDone && groundSamplesScored == 2){
                                    //waitSeconds(.01); //.1
                                    back3PathDone = true;
                                    bot.followPath(Sample3Back);
                                } else {
                                    //waitSeconds(.05); //.1
                                    //intake.stop();
                                    if (groundSamplesScored == 1) {
                                        updateScoreStart(2);
                                    } else if (groundSamplesScored == 2) {
                                        updateScoreStart(3);
                                    }

                                    arm.setTarget(5650, 4000);
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
                                waitMilliSeconds(500);
                                groundSamplesScored++;

                                if(groundSamplesScored < cycleCount) {
                                    if (groundSamplesScored == 1) {
                                        currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                        bot.setMaxPower(1); //.9
                                        bot.followPath(Sample2, true);
                                        arm.setTarget(0, 0);
                                        intake.pickup();
                                        pathTimer.reset();
                                        break;
                                    } else if (groundSamplesScored == 2) {
                                        currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                        bot.setMaxPower(1); //.9
                                        bot.followPath(Sample3, true);
                                        arm.setTarget(0, 0);
                                        intake.pickup();
                                        pathTimer.reset();
                                        break;
                                    }
                                } else {
                                    currentState = Sample_Auto.State.PARK;
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
                                currentState = Sample_Auto.State.END;
                            }
                    }
                } else if(AutoPose == AutoPoses.RIGHT){
                                            switch(currentState){
                        case SCORE_PASSIVE:
                            if (!bot.isBusy()) {
                                //intake.stop();
                                waitSeconds(.1);
                                //intake.drop();
                                waitSeconds(.1);
                                //intake.stop();

                                currentState = State.GET_GROUND_SAMPLE;
                                bot.followPath(Sample1, true);
                                pathTimer.reset();
                                break;

                            };
                        case GET_GROUND_SAMPLE:
                            //TODO add updateScoreStart()
                            if (!bot.isBusy()) {
                                //intake.pickup();
                                waitSeconds(1);
                                //intake.stop();

                                if(groundSamplesScored == 1){
                                    updateScoreStart(2);
                                } else if(groundSamplesScored == 2){
                                    updateScoreStart(3);
                                }

                                currentState = State.DROPOFF_SAMPLE;
                                bot.followPath(SampleDropoff, true);
                                pathTimer.reset();
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
                                pathTimer.reset();
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
                                    pathTimer.reset();
                                    break;
                                } else if(groundSamplesScored == 2){
                                    //sample3
                                    currentState = State.GET_GROUND_SAMPLE;
                                    bot.followPath(Sample3);
                                    pathTimer.reset();
                                    break;
                                } else {
                                    //park
                                    currentState = State.PARK;
                                    bot.followPath(Park);
                                    pathTimer.reset();
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