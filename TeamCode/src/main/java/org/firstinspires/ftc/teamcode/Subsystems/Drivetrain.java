//helps Android Studio know where file is located within our Repository
package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Utilities.BoundedArea;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.tuning.FollowerConstants;

public class Drivetrain {
    //motor objects
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private NormalizedColorSensor sideSensor;
    BoundedArea specimenScoreRegion;

    //FTC Lib & Pedro-Pathing objects
    Follower pedroDrivetrain;
    boolean teleOpDrive = true;
    PathChain scorePath;
    PathChain pickupPath;

    GamepadEx driverOp;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection;
    //Telemetry Data
    double strafeSpeed = 0;
    double forwardSpeed = 0;
    double turnSpeed = 0;
    int strength = 0;
    boolean robotCentricMode = true;
    double speedLimit = 1;
    double prevLimit = 1;


    ToggleButtonReader y_button, a_button; //drive modes
    ToggleButtonReader d_up, d_down, d_left, d_right; //speed control
    ToggleButtonReader left_bumper, right_bumper; //auto path

    //initializes the drivetrain
    public void init(HardwareMap hardwareMap, GamepadEx gamepad){
        pedroDrivetrain = new Follower(hardwareMap);

        //sideSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        //imu = hardwareMap.get(IMU.class, "imu");
        driverOp = gamepad;

        y_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.Y
        );
        a_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );
        d_up = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_UP
        );
        d_down = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_DOWN
        );
        d_left = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_LEFT
        );
        d_right = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_RIGHT
        );

        left_bumper = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.LEFT_BUMPER
        );

        right_bumper = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_BUMPER
        );


        //adding snappy breaking to drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pedroDrivetrain.startTeleopDrive();

//        if(PoseStorage.CurrentPose != null){ //uses last known pose from Auto as start in TeleOp
//            pedroDrivetrain.setPose(PoseStorage.CurrentPose);
//        }

        //TODO
        specimenScoreRegion = new BoundedArea(0,0,0,0, BoundedArea.AreaType.REGION);
    }

    public void init(HardwareMap hardwareMap, GamepadEx gamepad, Pose startPose){
        pedroDrivetrain = new Follower(hardwareMap);
        pedroDrivetrain.setPose(startPose);
        //sideSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        driverOp = gamepad;

        y_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.Y
        );
        a_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );
        d_up = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_UP
        );
        d_down = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_DOWN
        );
        d_left = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_LEFT
        );
        d_right = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_RIGHT
        );

        left_bumper = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.LEFT_BUMPER
        );

        right_bumper = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_BUMPER
        );


        //adding snappy breaking to drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pedroDrivetrain.startTeleopDrive();


        specimenScoreRegion = new BoundedArea(57,-30+72,14+72,-20+72, BoundedArea.AreaType.REGION);
    }


    private void updateToggles(){
        y_button.readValue();
        a_button.readValue();
        d_up.readValue();
        d_right.readValue();
        d_left.readValue();
        d_down.readValue();
        left_bumper.readValue();
        right_bumper.readValue();
    }

    private void BuildScorePath(Pose pose){
        scorePath = pedroDrivetrain.pathBuilder()
                .addPath(new BezierLine(pose.getPoint(),PoseStorage.LeftBucketTeleScore.getPoint()))
                .setLinearHeadingInterpolation(pose.getHeading(),PoseStorage.LeftBucketTeleScore.getHeading())
                .build();
    }

    private void BuildPickupPath(Pose pose){
        pickupPath = pedroDrivetrain.pathBuilder()
                .addPath(new BezierLine(pose.getPoint(),PoseStorage.LeftPitSamples.getPoint()))
                .setLinearHeadingInterpolation(pose.getHeading(),PoseStorage.LeftPitSamples.getHeading())
                .build();
    }

    public void run_teleOp(Driver_Feedback feedback){
       if(teleOpDrive) {
            strafeSpeed = -driverOp.getLeftX() * speedLimit; //changed to negative to fix inverted controls
            forwardSpeed = driverOp.getLeftY() * speedLimit;
            turnSpeed = -driverOp.getRightX() * speedLimit;

            double strafeAbsolute = Math.abs(strafeSpeed);
            double forwardAbsolute = Math.abs(forwardSpeed);
            double turnAbsolute = Math.abs(turnSpeed);



            //Robot Centric and Field Centric Modes
            if (y_button.wasJustPressed()) { //robot
                robotCentricMode = true;
            } else if (a_button.wasJustPressed()) { //field
                robotCentricMode = false;
            }




            //TODO - need to test before implementation
            if(left_bumper.wasJustPressed()){
                pedroDrivetrain.stopTeleOpDrive();
                teleOpDrive = false;
                pedroDrivetrain.setMaxPower(speedLimit);
                BuildScorePath(pedroDrivetrain.getPose());
                pedroDrivetrain.followPath(scorePath,true);
            }
            else if(right_bumper.wasJustPressed()){
                pedroDrivetrain.stopTeleOpDrive();
                teleOpDrive = false;
                pedroDrivetrain.setMaxPower(speedLimit);
                BuildPickupPath(pedroDrivetrain.getPose());
                pedroDrivetrain.followPath(pickupPath,true);
            }

            //Speed Controls - set top speed percentage
            if (d_up.wasJustPressed()) {
                speedLimit = 1;
            } else if (d_right.wasJustPressed()) {
                speedLimit = 0.7;
            } else if (d_left.wasJustPressed()) {
                speedLimit = 0.5;
            } else if (d_down.wasJustPressed()) {
                speedLimit = 0.3;
            }


            //Pedro-Pathing TeleOp Control
            pedroDrivetrain.setTeleOpMovementVectors(forwardSpeed, strafeSpeed, turnSpeed, robotCentricMode);
        } else {
            //logic to get out of autoDrive
            if(Math.abs(driverOp.getLeftX()) > 0 ||Math.abs(driverOp.getRightX()) > 0 || Math.abs(driverOp.getLeftY()) > 0){
                pedroDrivetrain.breakFollowing();
                teleOpDrive = true;
                pedroDrivetrain.setMaxPower(1);
                pedroDrivetrain.startTeleopDrive();
            }
        }


        pedroDrivetrain.update();
        updateToggles();

        //TODO Driver Alerts for Speed
//        if(strafeAbsolute == 1 || forwardAbsolute == 1 || turnAbsolute == 1){
//            strength = 100;
//            feedback.alert_driver(driverOp, strength);
//        } else if (strafeAbsolute > 0.9 || forwardAbsolute > 0.9 || turnAbsolute > 0.9) {
//            strength = 70;
//            feedback.alert_driver(driverOp, strength);
//        } else if (strafeAbsolute > 0.8 || forwardAbsolute > 0.8 || turnAbsolute > 0.8) {
//            strength = 40;
//            feedback.alert_driver(driverOp, strength);
//        } else {
//            strength = 0;
//        }
    }

    public void run_teleOp2(){
        // if(teleOpDrive) {
        strafeSpeed = -driverOp.getLeftX() * speedLimit; //changed to negative to fix inverted controls
        forwardSpeed = driverOp.getLeftY() * speedLimit;
        turnSpeed = -driverOp.getRightX() * speedLimit;

        double strafeAbsolute = Math.abs(strafeSpeed);
        double forwardAbsolute = Math.abs(forwardSpeed);
        double turnAbsolute = Math.abs(turnSpeed);


        //Robot Centric and Field Centric Modes
        if (y_button.wasJustPressed()) { //robot
            robotCentricMode = true;
        } else if (a_button.wasJustPressed()) { //field
            robotCentricMode = false;
        }

        //Constant Path Creation + Controls
//        if(PoseStorage.allianceSide == AutoBase.AutoPoses.LEFT){
//            //create left side paths
//            scorePath = new Path(new BezierLine(pedroDrivetrain.getPose().getPoint(), PoseStorage.LeftBucketScore.getPoint()));
//            scorePath.setLinearHeadingInterpolation(pedroDrivetrain.getPose().getHeading(), PoseStorage.LeftBucketScore.getHeading());
//            pickupPath = new Path(new BezierLine(pedroDrivetrain.getPose().getPoint(), PoseStorage.LeftPitSamples.getPoint()));
//            pickupPath.setLinearHeadingInterpolation(pedroDrivetrain.getPose().getHeading(), PoseStorage.LeftPitSamples.getHeading());
//        } else {
//            //create right side paths
//            scorePath = new Path(new BezierLine(pedroDrivetrain.getPose().getPoint(), PoseStorage.SpecimenScore.getPoint()));
//            scorePath.setLinearHeadingInterpolation(pedroDrivetrain.getPose().getHeading(), PoseStorage.SpecimenScore.getHeading());
//            pickupPath = new Path(new BezierLine(pedroDrivetrain.getPose().getPoint(), PoseStorage.RightPitSamples.getPoint()));
//            pickupPath.setLinearHeadingInterpolation(pedroDrivetrain.getPose().getHeading(), PoseStorage.RightPitSamples.getHeading());
//        }

        //TODO - need to test before implementation
//            if(left_bumper.wasJustPressed()){
//                pedroDrivetrain.stopTeleOpDrive();
//                teleOpDrive = false;
//                pedroDrivetrain.setMaxPower(speedLimit);
//                pedroDrivetrain.followPath(scorePath,true);
//            } else if(right_bumper.wasJustPressed()){
//                pedroDrivetrain.stopTeleOpDrive();
//                teleOpDrive = false;
//                pedroDrivetrain.setMaxPower(speedLimit);
//                pedroDrivetrain.followPath(pickupPath,true);
//            }

        //Speed Controls - set top speed percentage
        if (d_up.wasJustPressed()) {
            speedLimit = 1;
            prevLimit = 1;
        } else if (d_right.wasJustPressed()) {
            speedLimit = 0.7;
            prevLimit = 0.7;
        } else if (d_left.wasJustPressed()) {
            speedLimit = 0.5;
            prevLimit = 0.5;
        } else if (d_down.wasJustPressed()) {
            speedLimit = 0.3;
            prevLimit = 0.3;
        }


        //Pedro-Pathing TeleOp Control
        pedroDrivetrain.setTeleOpMovementVectors(forwardSpeed, strafeSpeed, turnSpeed, robotCentricMode);
//        } else {
//            //logic to get out of autoDrive
//            if(Math.abs(driverOp.getLeftX()) > 0 ||Math.abs(driverOp.getRightX()) > 0 || Math.abs(driverOp.getLeftY()) > 0){
//                pedroDrivetrain.breakFollowing();
//                teleOpDrive = true;
//                pedroDrivetrain.setMaxPower(1);
//                pedroDrivetrain.startTeleopDrive();
//            }
//        }

        pedroDrivetrain.update();

        //TODO specimen scoring distance control for Edgar
        //Method #1 - Distance Sensor - slows speed down when distance sensor is less than certain range



        //Method #2 - Odometry Perception - slow speed down when coordinate is within bounded area
        if(pedroDrivetrain.isWithinArea(specimenScoreRegion)){
            if(robotCentricMode){
                //strafe right
                if(driverOp.getLeftX() > 0){
                    speedLimit = 0.3; //30%
                }
            } else {
                //move up
                if(driverOp.getLeftY() > 0){
                    speedLimit = 0.3; //30%
                }
            }
        } else {
            speedLimit = prevLimit;
        }


        updateToggles();
    }


    //most important info of drivetrain to reduce clutter
    public void getTelemetry(Telemetry telemetry){
        telemetry.addLine("---Drivetrain Control Data---");
        telemetry.addData("Strafe Speed: ", strafeSpeed);
        telemetry.addData("Forward Speed: ", turnSpeed);
        telemetry.addData("Turn Speed: ", turnSpeed);
    }

    public void getTelemetry2(Telemetry telemetry){
        telemetry.addLine("---Drivetrain Control Data---");
        telemetry.addData("Within Specimen Scoring Area?: ", pedroDrivetrain.isWithinArea(specimenScoreRegion));
        telemetry.addData("Strafe Speed: ", strafeSpeed);
        telemetry.addData("Forward Speed: ", turnSpeed);
        telemetry.addData("Turn Speed: ", turnSpeed);
    }
}