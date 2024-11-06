package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Utilities.PoseStoragePedro;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.List;

//this class is used by all auto programs to access subsystem controls & AutoMenu
public class AutoBase extends LinearOpMode {
    //subsystem objects - arm, intake, lift, etc
    protected org.firstinspires.ftc.teamcode.Subsystems.LED lighting = new LED();

    //auto pathing variables and arrays to loop through
    ToggleButtonReader d_up,d_down,d_left,d_right;


    public enum AutoPoses {
        LEFT,
        RIGHT
    }

    public Pose[] Coordinates =
            {PoseStoragePedro.LeftSample1,
                    PoseStoragePedro.LeftSample2,
                    PoseStoragePedro.LeftSample3,
                    PoseStoragePedro.LeftBucketScore,
                    PoseStoragePedro.PitSamples,
                    PoseStoragePedro.LeftCheckPoint,
                    PoseStoragePedro.RightSample1,
                    PoseStoragePedro.RightSample2,
                    PoseStoragePedro.RightSample3,
                    PoseStoragePedro.SampleDropoff,
                    PoseStoragePedro.SpecimenPickup,
                    PoseStoragePedro.SpecimenScore,
                    PoseStoragePedro.RightCheckPoint
            };

    public String[] CoordinateNames =
            {"LeftSample1",
                    "LeftSample2",
                    "LeftSample3",
                    "LeftBucketScore",
                    "PitSamples",
                    "LeftCheckPoint",
                    "RightSample1",
                    "RightSample2",
                    "RightSample3",
                    "SampleDropoff",
                    "SpecimenPickup",
                    "SpecimenScore",
                    "RightCheckPoint"
            };

    public Pose currentCoordinate = Coordinates[0];
    public String currentCoordinateName = CoordinateNames[0];
    public AutoPoses AutoPose = AutoPoses.RIGHT; //default is right (samples)
    public int cycleCount = 1; //number of times to go to pit
    public int coordinateCount = 1;
    public boolean returnHome = false; //yes or no for returning to startPose during test autos

    //blank runOpMode() method included only to keep LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {}

    public void init_classes(){

    }

    /**
    * used by new auto programs to init subsystems
     */
    public void init_classes(GamepadEx driverOp){
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

    }

    /**
     * Runs the Main Auto Menu Program to input Auto Pathing Selections
     */
    public void runMenu(){
        //menu code
        if(d_up.getState()){
            //startPose selection
            AutoPose = AutoPoses.RIGHT;
        } else {
            AutoPose = AutoPoses.LEFT;
        }

        if(d_down.wasJustPressed()){
            //cycle count selection
            if(cycleCount > 2){
                cycleCount = 1;
            } else {
                cycleCount++;
            }
        }

        //menu
        telemetry.addLine("Select the StartPose by Toggling the D-pad Up Button");
        telemetry.addData("Current StartPose Selected: ", AutoPose);
        telemetry.addLine(" ");
        telemetry.addLine("Select the # of Cycles by pressing the D-pad Down Button");
        telemetry.addData("Number of cycles: ", cycleCount);
        //telemetry.update();

        //updates gamepad readers
        d_up.readValue();
        d_down.readValue();
        d_left.readValue();
        d_right.readValue();
    }


    /**
     * A Specialized Auto Menu Program for the "CoordinateTester" Program
     */
    public void runCoordinateMenu(){
        //menu code
        if(d_up.getState()){
            //startPose selection
            AutoPose = AutoPoses.RIGHT;
        } else {
            AutoPose = AutoPoses.LEFT;
        }

        //Home Return Selection
        returnHome = !d_down.getState();

        if(d_left.wasJustPressed()){
            //coordinate - count selection
            if(coordinateCount < 2){
                coordinateCount = 13;
            } else {
                coordinateCount--;
            }

            currentCoordinate = Coordinates[coordinateCount-1];
            currentCoordinateName = CoordinateNames[coordinateCount-1];

        } else if(d_right.wasJustPressed()){
            //coordinate + count selection
            if(coordinateCount > 12){
                coordinateCount = 1;
            } else {
                coordinateCount++;
            }

            currentCoordinate = Coordinates[coordinateCount-1];
            currentCoordinateName = CoordinateNames[coordinateCount-1];
        }

        //menu
        telemetry.addLine("Select the StartPose by Toggling the D-pad Up Button");
        telemetry.addData("Current StartPose Selected: ", AutoPose);
        telemetry.addLine(" ");
        telemetry.addLine("Select the Target Coordinate by using the Left and Right D-pads");
        telemetry.addData("Selected Coordinate: ", currentCoordinateName);
        telemetry.addLine(" ");
        telemetry.addLine("Select if bot should return to start by Toggling the D-pad Down Button");
        telemetry.addData("Return to Start?: ", returnHome);
        //TODO add telemetry for coordinate
        //telemetry.update();

        //updates gamepad readers
        d_up.readValue();
        d_down.readValue();
        d_left.readValue();
        d_right.readValue();
    }

    public AutoPoses getAutoPose(){
        return AutoPose;
    }

    public int getCycleCount(){
        return cycleCount;
    }

    public boolean getReturnHome(){
        return returnHome;
    }

    public Pose getCurrentCoordinate(){
        return currentCoordinate;
    }
}
