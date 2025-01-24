package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;

public class PoseStorage {
    //**Reminder that Pedro-Pathing's Origin is at the bottom left of the field
    //the maximum distance in each axis in 140" (inches)
    //adding +72" to each coordinate converts RR Poses to PP Poses

    //Equal X & Y change of 0.707 equates to a forward and backward change of 1 inch
    //add or subtract method below to x & y

    /**
     * positive is forward, negative is backward
     * @param inches forward
     * @return
     */
    private static double OneInchMovement(double inches){
        return -0.707 * inches;
    }

    //dynamic pose variable to save current bot position from auto to teleOp (teleOp will use enhances from Pedro-Pathing)
    public static Pose CurrentPose = null;

    public static AutoBase.AutoPoses allianceSide = AutoBase.AutoPoses.LEFT; //useful variable for selected alliance from auto

    //used to determine arm pose (auto is on rung, no auto is 0)
    public static boolean ranAuto = false;

    //Left Side Poses - Samples
    public static Pose LeftStartPose = new Pose(37,12, Math.toRadians(90)); //Done

    //Sample 1 is outside, Sample 3 is inside
    public static Pose LeftSample1 = new Pose(37,49.2, Math.toRadians(180-1e-6)); //Done / Previous -39+72,-24+72, Math.toRadians(180-1e-6)

    public static Pose LeftSample2 = new Pose(25.7,49.2, Math.toRadians(180-1e-6)); //Done

    public static Pose LeftSample3 = new Pose(21,49.2,Math.toRadians(180-1e-6)); //Done

    public static Pose LeftSample3Back = new Pose(37,48,Math.toRadians(180-1e-6)); //Done

    public static Pose LeftSampleControlPose = new Pose(36.1,48,Math.toRadians(180-1e-6)); //done

    public static Pose LeftBucketScore = new Pose(21-(OneInchMovement(-5)),-50-(OneInchMovement(-3.8))+72,Math.toRadians(45)); //Done - 225

    public static Pose LeftPitSamples = new Pose(-18+72,72,Math.toRadians(0)); //done

    public static Pose LeftCheckPoint = new Pose(-55+72,-48+72, Math.toRadians(225)); //DONE

    public static Pose LeftPark = new Pose(48,69, Math.toRadians(0)); //Done - 83 y



    //Right Side Poses - Specimen
    public static Pose RightStartPose = new Pose(11+72,-60+72, Math.toRadians(90)); //Done

    public static Pose RightSample1 = new Pose(37+72,-35+72,Math.toRadians(45)); //DONE 35

    public static Pose RightSample2 = new Pose(49+72,-35+72,Math.toRadians(45)); //Done

    public static Pose RightSample3 = new Pose(56+72,-35+72,Math.toRadians(45)); //Done

    public static Pose SampleDropoff = new Pose(52+72,-50+72,Math.toRadians(-55)); //done - 40,-49,180

    public static Pose SpecimenPickup = new Pose(43+72,-50+72,Math.toRadians(-90)); //done - 50, -56

    public static Pose SpecimenScore = new Pose(0+72,-35+72,Math.toRadians(90)); //done

    public static Pose RightPitSamples = new Pose(90,72,Math.toRadians(180-1e-6)); //TODO

    public static Pose RightCheckPoint = new Pose(19+72,-39+72, Math.toRadians(180)); //TODO

    public static Pose RightPark = new Pose(35+72,-50+72, Math.toRadians(90)); //Done - (-31), 51, 0




}