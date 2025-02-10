package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;

@Config
@Autonomous(name="Coordinate Tester", group = "Autos")
public class CoordinateTester extends AutoBase{

    //important variables
    PathChain startToTarget, targetToStart, adjustPath;
    Pose startPose, targetPose;
    String targetPoseName;
    Follower bot;
    AutoPoses AutoPose;
    boolean returnHome;
    GamepadEx driverOp;
    ToggleButtonReader a_button, d_up, d_down, d_left, d_right;
    TriggerReader leftTrigger, rightTrigger;
    public int prevArmTarget = 0;
    public static int armTarget = 0;
    public int prevExtendTarget = 0;
    public static int extendTarget = 0;

    //Finite State Machine (FSM) variables
    public enum State {
        START, //go to target//move with d_pad
        RETURN, //return to startPose
        END //end of program
    }

    public enum ADJUST_TYPE {
        LEFT,
        RIGHT,
        UP,
        DOWN,
        ROTATE_LEFT,
        ROTATE_RIGHT
    }

    public State currentState;

    //add all paths/auto objectives here
    public void buildPaths(AutoPoses AutoPose){
        if(AutoPose == AutoPoses.LEFT){
            startPose = PoseStorage.LeftStartPose;
        } else if (AutoPose == AutoPoses.RIGHT) {
            startPose = PoseStorage.RightStartPose;
        }

        startToTarget = bot.pathBuilder()
                .addPath(new BezierLine(startPose.getPoint(), targetPose.getPoint()))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();

        targetToStart = bot.pathBuilder()
                .addPath(new BezierLine(targetPose.getPoint(), startPose.getPoint()))
                .setLinearHeadingInterpolation(targetPose.getHeading(), startPose.getHeading())
                .build();
    }

    public void buildAdjustPath(ADJUST_TYPE type){
        switch(type){
            case LEFT:
                //create adjustPath
                adjustPath = bot.pathBuilder()
                        .addPath(new BezierLine(bot.getPose().getPoint(), new Pose(bot.getPose().getX()-3, bot.getPose().getY(), bot.getPose().getHeading()).getPoint()))
                        .setConstantHeadingInterpolation(bot.getPose().getHeading())
                        .build();
                //update returnPath
                targetToStart  = bot.pathBuilder()
                        .addPath(new BezierLine(new Pose(bot.getPose().getX()-3, bot.getPose().getY(), bot.getPose().getHeading()).getPoint(), startPose.getPoint()))
                        .setLinearHeadingInterpolation(bot.getPose().getHeading(), startPose.getHeading())
                        .build();
                break;
            case RIGHT:
                //create adjustPath
                adjustPath = bot.pathBuilder()
                        .addPath(new BezierLine(bot.getPose().getPoint(), new Pose(bot.getPose().getX()+3, bot.getPose().getY(), bot.getPose().getHeading()).getPoint()))
                        .setConstantHeadingInterpolation(bot.getPose().getHeading())
                        .build();
                //update returnPath
                targetToStart  = bot.pathBuilder()
                        .addPath(new BezierLine(new Pose(bot.getPose().getX()+3, bot.getPose().getY(), bot.getPose().getHeading()).getPoint(), startPose.getPoint()))
                        .setLinearHeadingInterpolation(bot.getPose().getHeading(), startPose.getHeading())
                        .build();
                break;
            case UP:
                //create adjustPath
                adjustPath = bot.pathBuilder()
                        .addPath(new BezierLine(bot.getPose().getPoint(), new Pose(bot.getPose().getX(), bot.getPose().getY()+3, bot.getPose().getHeading()).getPoint()))
                        .setConstantHeadingInterpolation(bot.getPose().getHeading())
                        .build();
                //update returnPath
                targetToStart  = bot.pathBuilder()
                        .addPath(new BezierLine(new Pose(bot.getPose().getX(), bot.getPose().getY()+3, bot.getPose().getHeading()).getPoint(), startPose.getPoint()))
                        .setLinearHeadingInterpolation(bot.getPose().getHeading(), startPose.getHeading())
                        .build();
                break;
            case DOWN:
                //create adjustPath
                adjustPath = bot.pathBuilder()
                        .addPath(new BezierLine(bot.getPose().getPoint(), new Pose(bot.getPose().getX(), bot.getPose().getY()-3, bot.getPose().getHeading()).getPoint()))
                        .setConstantHeadingInterpolation(bot.getPose().getHeading())
                        .build();
                //update returnPath
                targetToStart  = bot.pathBuilder()
                        .addPath(new BezierLine(new Pose(bot.getPose().getX(), bot.getPose().getY()-3, bot.getPose().getHeading()).getPoint(), startPose.getPoint()))
                        .setLinearHeadingInterpolation(bot.getPose().getHeading(), startPose.getHeading())
                        .build();
                break;
            case ROTATE_LEFT:
                //create adjustPath
                adjustPath = bot.pathBuilder()
                        .addPath(new BezierLine(bot.getPose().getPoint(), new Pose(bot.getPose().getX(), bot.getPose().getY(), bot.getPose().getHeading()-Math.toRadians(10)).getPoint()))
                        .setLinearHeadingInterpolation(bot.getPose().getHeading(), bot.getPose().getHeading()-Math.toRadians(10))
                        .build();
                //update returnPath
                targetToStart  = bot.pathBuilder()
                        .addPath(new BezierLine(new Pose(bot.getPose().getX(), bot.getPose().getY()-3, bot.getPose().getHeading()-Math.toRadians(10)).getPoint(), startPose.getPoint()))
                        .setLinearHeadingInterpolation(bot.getPose().getHeading(), bot.getPose().getHeading()-Math.toRadians(10))
                        .build();
                break;
            case ROTATE_RIGHT:
                //create adjustPath
                adjustPath = bot.pathBuilder()
                        .addPath(new BezierLine(bot.getPose().getPoint(), new Pose(bot.getPose().getX(), bot.getPose().getY(), bot.getPose().getHeading()+Math.toRadians(10)).getPoint()))
                        .setLinearHeadingInterpolation(bot.getPose().getHeading(), bot.getPose().getHeading()-Math.toRadians(10))
                        .build();
                //update returnPath
                targetToStart  = bot.pathBuilder()
                        .addPath(new BezierLine(new Pose(bot.getPose().getX(), bot.getPose().getY(), bot.getPose().getHeading()+Math.toRadians(10)).getPoint(), startPose.getPoint()))
                        .setLinearHeadingInterpolation(bot.getPose().getHeading(), bot.getPose().getHeading()-Math.toRadians(10))
                        .build();
                break;
        }

    }

    public void runOpMode() throws InterruptedException{
        bot = new Follower(hardwareMap);

        startPose = PoseStorage.LeftStartPose;
        targetPose = Coordinates[0];
        targetPoseName = CoordinateNames[0];

        driverOp = new GamepadEx(gamepad1);

        //initialize subsystems
        init_classes(driverOp);

        a_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );
        d_down = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_DOWN
        );
        d_up = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_UP
        );
        d_left = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_LEFT
        );
        d_right = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_RIGHT
        );
        leftTrigger = new TriggerReader(
                driverOp, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        rightTrigger = new TriggerReader(
                driverOp, GamepadKeys.Trigger.RIGHT_TRIGGER
        );


        //run Auto Menu
        while(opModeInInit()){
            runCoordinateMenu();
            AutoPose = getAutoPose();
            targetPose = getSelectedCoordinate();
            targetPoseName = getSelectedCoordinateName();
            returnHome = getReturnHome();
            telemetry.update();
        }

        waitForStart();

        buildPaths(AutoPose);

        bot.setPose(startPose);

        //starting path & FSM
        currentState = State.START;
        bot.followPath(startToTarget);

        while(opModeIsActive()){
            //FSM Auto Logic
            //arm.runPassiveExtend();
            switch(currentState){
                case START:
                    if(!bot.isBusy()){
                        bot.holdPoint(new BezierPoint(targetPose.getPoint()), targetPose.getHeading());
                        if(a_button.wasJustPressed() && returnHome) {
                            currentState = State.RETURN;
                            bot.followPath(targetToStart);
                            arm.setTarget(0,0);

                        } else if(d_left.wasJustPressed()){ //inch to the left
                            buildAdjustPath(ADJUST_TYPE.LEFT);
                            bot.followPath(adjustPath);

                        } else if(d_right.wasJustPressed()){ //inch to the right
                            buildAdjustPath(ADJUST_TYPE.RIGHT);
                            bot.followPath(adjustPath);

                        } else if(d_down.wasJustPressed()){ //inch down
                            buildAdjustPath(ADJUST_TYPE.DOWN);
                            bot.followPath(adjustPath);

                        } else if(d_up.wasJustPressed()){ //inch up
                            buildAdjustPath(ADJUST_TYPE.UP);
                            bot.followPath(adjustPath);

                        } else if(leftTrigger.wasJustPressed()){
                            buildAdjustPath(ADJUST_TYPE.ROTATE_LEFT); // rotate 10° counterclockwise (left)
                            bot.followPath(adjustPath);

                        } else if(rightTrigger.wasJustPressed()){
                            buildAdjustPath(ADJUST_TYPE.ROTATE_RIGHT); // rotate 10° clockwise (right)
                            bot.followPath(adjustPath);
                        }
                        break;
                    }
                case RETURN:
                    if(!bot.isBusy()) {
                        currentState = State.END;
                    }
            }

            if(prevArmTarget != armTarget){
                prevArmTarget = armTarget;
                arm.setTarget(armTarget,extendTarget);
            }
            if(prevExtendTarget != extendTarget){
                prevExtendTarget = extendTarget;
                arm.setTarget(armTarget,extendTarget);
            }

            bot.update(); //controls Pedro-Pathing logic
            arm.update();

            PoseStorage.CurrentPose = bot.getPose(); //updates currentPose variable
            telemetry.addData("Selected Auto Position: ", AutoPose);
            telemetry.addData("Selected Target Position: ", targetPoseName);
            telemetry.addData("Current State: ", currentState);
            telemetry.addData("X Position: ", bot.getPose().getX());
            telemetry.addData("Y Position: ", bot.getPose().getY());
            telemetry.addData("Heading Position: ", bot.getPose().getHeading());
            telemetry.addData("A Button State: ", a_button.getState());
            arm.getTelemetry(telemetry);

            telemetry.update();
            a_button.readValue();
            d_up.readValue();
            d_down.readValue();
            d_left.readValue();
            d_right.readValue();
            leftTrigger.readValue();
            rightTrigger.readValue();
        }
    }
}
