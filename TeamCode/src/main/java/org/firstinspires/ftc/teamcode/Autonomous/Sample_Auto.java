package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.PoseStoragePedro;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Point;

@Autonomous(name="3 Sample Auto", group = "Autos")
public class Sample_Auto extends AutoBase {
    //important variables
    PathChain scorePassive, Sample1, Sample2, Sample3, score, park;
    Pose startPose;
    Pose parkPose;
    Follower bot;
    AutoPoses AutoPose = getAutoPose();
    int inverseConstant = 1; //set to 1 or -1 depending on AutoPose - used only for parking
    GamepadEx driverOp;

    //Finite State Machine (FSM) variables
    public enum State {
        SCORE_PASSIVE,
        GET_GROUND_SAMPLE,
        SCORE,
        PARK,
        END
    }

    public State currentState;
    public int groundSamplesScored = 0;

    //add all paths/auto objectives here
    public void buildPaths(AutoPoses AutoPose){
        if(AutoPose == AutoPoses.LEFT){
            scorePassive = bot.pathBuilder()
                    .build();

            Sample1 = bot.pathBuilder()
                    .build();

            Sample2 = bot.pathBuilder()
                    .build();

            Sample3 = bot.pathBuilder()
                    .build();

            score = bot.pathBuilder()
                    .build();

            park = bot.pathBuilder()
                    .build();

        } else if(AutoPose == AutoPoses.RIGHT){
            scorePassive = bot.pathBuilder()
                    //drop loaded sample in observation szone

                    //drive to checkpoint
                    .build();

            Sample1 = bot.pathBuilder()
                    //pickup sample 1

                    //drop in observation zone

                    //pickup specimen

                    //return to checkpoint
                    .build();

            Sample2 = bot.pathBuilder()
                    .build();

            Sample3 = bot.pathBuilder()
                    .build();

            score = bot.pathBuilder()
                    .build();

            park = bot.pathBuilder()
                    .build();
        }
    }

    public void runOpMode() throws InterruptedException{
        bot = new Follower(hardwareMap);

        startPose = PoseStoragePedro.LeftStartPose;
        parkPose = PoseStoragePedro.LeftPark;

        driverOp = new GamepadEx(gamepad1);

        //initialize subsystems
        init_classes(driverOp);


        while(opModeInInit()){
            runMenu();
            AutoPose = getAutoPose();
            telemetry.update();
        }

        waitForStart();

        buildPaths(AutoPose); //builds paths after we select the autoStart pose from the menu

        bot.setPose(startPose);

        // starting path & FSM
        currentState = State.SCORE_PASSIVE;
        bot.followPath(scorePassive);

        while(opModeIsActive()){
            // FSM Auto Logic
            switch(currentState){
                case SCORE_PASSIVE:
                    if(!bot.isBusy()){
                        currentState = State.GET_GROUND_SAMPLE;
                        bot.followPath(Sample1);
                        break;
                    }
                case GET_GROUND_SAMPLE:
                    if(!bot.isBusy()){
                        currentState = State.SCORE;
                        bot.followPath(score);
                        break;
                    }
                case SCORE:
                    groundSamplesScored++;
                    if(!bot.isBusy()){
                        if(groundSamplesScored == 1){
                            currentState = State.GET_GROUND_SAMPLE;
                            bot.followPath(Sample2);
                            break;
                        } else if(groundSamplesScored == 2){
                            currentState = State.GET_GROUND_SAMPLE;
                            bot.followPath(Sample3);
                            break;
                        } else {
                            currentState = State.PARK;
                            bot.followPath(park);
                            break;
                        }
                    }
                case PARK:
                    if(!bot.isBusy()){
                        currentState = State.END;
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
