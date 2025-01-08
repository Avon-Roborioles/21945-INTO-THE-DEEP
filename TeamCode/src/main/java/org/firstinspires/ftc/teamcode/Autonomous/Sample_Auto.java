package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Utilities.PoseStoragePedro;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.util.Drawing;

import java.util.concurrent.TimeUnit;

@Autonomous(name="3 Sample Auto", group = "Autos")
public class Sample_Auto extends AutoBase {
    //important variables
    Path scorePassive, Score, Park, SampleDropoff, PickupSpecimen;
    PathChain scorePassiveChain, Sample1, Sample2, Sample3;
    Pose startPose;
    Pose parkPose;
    Follower bot;
    AutoPoses AutoPose = getAutoPose();
    GamepadEx driverOp;
    String message = "";
    ElapsedTime pathTimer;


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

            Sample1 = bot.pathBuilder() //drives to control pose first then leftSample1 to scoop sample
                    .addPath(new BezierLine(PoseStoragePedro.LeftBucketScore.getPoint(), PoseStoragePedro.LeftSampleControlPose.getPoint()))
                    .setLinearHeadingInterpolation(PoseStoragePedro.LeftBucketScore.getHeading(), PoseStoragePedro.LeftSampleControlPose.getHeading())
                    .setPathEndVelocityConstraint(10)
                    .addPath(new BezierLine(PoseStoragePedro.LeftSampleControlPose.getPoint(), PoseStoragePedro.LeftSample1.getPoint()))
                    .setConstantHeadingInterpolation(PoseStoragePedro.LeftSampleControlPose.getHeading())
                    .build();


            Sample2 = bot.pathBuilder() //drives to sample1 pose first then sample2 pose to scoop sample
                    .addPath(new BezierLine(PoseStoragePedro.LeftBucketScore.getPoint(), PoseStoragePedro.LeftSample1.getPoint()))
                    .setLinearHeadingInterpolation(PoseStoragePedro.LeftBucketScore.getHeading(), PoseStoragePedro.LeftSample1.getHeading())
                    .setPathEndVelocityConstraint(10)
                    .addPath(new BezierLine(PoseStoragePedro.LeftSample1.getPoint(), PoseStoragePedro.LeftSample2.getPoint()))
                    .setConstantHeadingInterpolation(PoseStoragePedro.LeftSample1.getHeading())
                    .build();


            Sample3 = bot.pathBuilder() //drives to sample2 pose first then sample3 to scoop sample
                    .addPath(new BezierLine(PoseStoragePedro.LeftBucketScore.getPoint(), PoseStoragePedro.LeftSample2.getPoint()))
                    .setLinearHeadingInterpolation(PoseStoragePedro.LeftBucketScore.getHeading(), PoseStoragePedro.LeftSample2.getHeading())
                    .setPathEndVelocityConstraint(10)
                    .addPath(new BezierLine(PoseStoragePedro.LeftSample2.getPoint(), PoseStoragePedro.LeftSample3.getPoint()))
                    .setConstantHeadingInterpolation(PoseStoragePedro.LeftSample2.getHeading())
                    .build();

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

            //Sample1 = new Path(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightSample1.getPoint()));
            //Sample1.setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightSample1.getHeading());
            Sample1 = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightSample1.getPoint()))
                    .setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightSample1.getHeading())
                    .build();

            //Sample2 = new Path(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightSample2.getPoint()));
            //Sample2.setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightSample2.getHeading());
            Sample2 = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightSample2.getPoint()))
                    .setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightSample2.getHeading())
                    .build();


            //Sample3 = new Path(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightSample3.getPoint()));
            //Sample3.setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightSample3.getHeading());
            Sample3 = bot.pathBuilder()
                    .addPath(new BezierLine(PoseStoragePedro.SpecimenScore.getPoint(), PoseStoragePedro.RightSample3.getPoint()))
                    .setLinearHeadingInterpolation(PoseStoragePedro.SpecimenScore.getHeading(), PoseStoragePedro.RightSample3.getHeading())
                    .build();



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

    //native wait method
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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    //various PID updating & Telemetry Data bundled into one method
    public void updateAuto(){
        bot.update(); //controls Pedro-Pathing logic
        subsystemsUpdate();
        //arm.runPassiveExtend();
        PoseStoragePedro.CurrentPose = bot.getPose(); //updates currentPose variable
        mainTelemetry.addLine("---AUTO DATA---");
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



    //auto loop
        public void runOpMode () throws InterruptedException {
            bot = new Follower(hardwareMap);

            startPose = PoseStoragePedro.LeftStartPose;
            parkPose = PoseStoragePedro.LeftPark;

            driverOp = new GamepadEx(gamepad1);

            pathTimer = new ElapsedTime();

            //initialize subsystems
            init_classes(driverOp);

            //init loop
            while (opModeInInit()) {
                runMenu(mainTelemetry);
                AutoPose = getAutoPose();
                mainTelemetry.update();
            }

            waitForStart();

            buildPaths(AutoPose); //builds paths after we select the autoStart pose from the menu

            if(AutoPose == AutoPoses.LEFT){
                bot.setPose(PoseStoragePedro.LeftStartPose);
            } else if(AutoPose == AutoPoses.RIGHT){
                bot.setPose(PoseStoragePedro.RightStartPose);
            }

            //Draw initial bot pose on field (FTC Dashboard)
            Drawing.drawRobot(bot.getPose(), "#4CAF50");
            Drawing.sendPacket();

            // starting path & FSM
            currentState = State.SCORE_PASSIVE;
            bot.followPath(scorePassive, true);
            intake.pickup(); //should secure passive/loaded sample
            arm.setTarget(4000,200);
            pathTimer.reset();


            while (opModeIsActive()) {
                // FSM Auto Logic
                if(AutoPose == AutoPoses.LEFT) {
                    switch(currentState) {
                        case SCORE_PASSIVE:
                            if (!bot.isBusy()) {
                                waitSeconds(0.1);
                                intake.drop(); //score
                                waitSeconds(1);
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
                                waitSeconds(.1);
                                //intake.stop();
                                if (groundSamplesScored == 1) {
                                    updateScoreStart(2);
                                } else if (groundSamplesScored == 2) {
                                    updateScoreStart(3);
                                }

                                currentState = State.SCORE;
                                bot.followPath(Score, true);
                                arm.setTarget(4000,200);
                                pathTimer.reset();
                                break;
                            }

                        case SCORE:
                            if (!bot.isBusy()) {
                                waitSeconds(0.1);
                                intake.drop();
                                waitSeconds(1);
                                groundSamplesScored++;

                                if (groundSamplesScored == 1) {
                                    currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                    bot.followPath(Sample2, true);
                                    arm.setTarget(0,0);
                                    intake.pickup();
                                    pathTimer.reset();
                                    break;
                                } else if (groundSamplesScored == 2) {
                                    currentState = Sample_Auto.State.GET_GROUND_SAMPLE;
                                    bot.followPath(Sample3, true);
                                    arm.setTarget(0,0);
                                    intake.pickup();
                                    pathTimer.reset();
                                    break;
                                } else {
                                    currentState = Sample_Auto.State.PARK;
                                    bot.followPath(Park, true);
                                    arm.setTarget(0,0); //TODO adjust this so bot touches ascend rung
                                    intake.stop();
                                    pathTimer.reset();
                                    break;
                                }
                            }
                        case PARK:
                            if (!bot.isBusy()) {
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