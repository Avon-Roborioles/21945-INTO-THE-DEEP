package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;

//Same Functionality as other PoseStorage but for Pedro-Pathing
//having a new file for this makes merging easier on Git

public class PoseStoragePedro {
    //**Reminder that Pedro-Pathing's Origin is at the bottom left of the field
    //the maximum distance in each axis in 140" (inches)
    //adding +72" to each coordinate converts RR Poses to PP Poses

    //dynamic pose variable to save current bot position from auto to teleOp (teleOp will use enhances from Pedro-Pathing)
    public Pose CurrentPose = null;

    //Left Side Poses - Samples
    public static Pose LeftStartPose = new Pose(-35+72,-60+72, Math.toRadians(90));

    public static Pose LeftSample1 = new Pose(-35+72,-39+72, Math.toRadians(135));

    public static Pose LeftSample2 = new Pose(-46+72,-39+72, Math.toRadians(135));

    public static Pose LeftSample3 = new Pose(-52+72,-25+72,Math.toRadians(179));

    public static Pose LeftBucketScore = new Pose(-49+72,-49+72,Math.toRadians(225));

    public static Pose LeftPitSamples = new Pose(-28+72,-9+72,Math.toRadians(0));

    public static Pose LeftPark = new Pose(-29+72,-9.8+72, Math.toRadians(0));



    //Right Side Poses - Specimen
    public static Pose RightStartPose = new Pose(11+72,-60+72, Math.toRadians(90));

    public static Pose RightSample1 = new Pose(35+72,-39+72,Math.toRadians(45));

    public static Pose RightSample2 = new Pose(46+72,-39+72,Math.toRadians(50));

    public static Pose RightSample3 = new Pose(52+72,-25+72,Math.toRadians(0));

    public static Pose RightBucketScore = new Pose(49+72,-49+72,Math.toRadians(225));

    public static Pose RightPitSamples = new Pose(-28+72,-9+72,Math.toRadians(179));

    public static Pose SampleDropoff = new Pose(51+72,-40+72,Math.toRadians(-45));

    public static Pose SpecimenPickup = new Pose(50+72,-57+72,Math.toRadians(90));

    public static Pose SpecimenScore = new Pose(0+72,-33+72,Math.toRadians(-90));

    public static Pose RightPark = new Pose(29+72,-9.8+72, Math.toRadians(180-1e-6));


    //useful points to hold during auto - allows us to keep our stance against aggressive bots
    //Left side
    public static Point LeftBucketScorePoint = LeftBucketScore.getPoint();

    public static Point LeftPitSamplesPoint = LeftPitSamples.getPoint();

    //Right side
    public static Point RightBucketScorePoint = RightBucketScore.getPoint();

    public static Point RightPitSamplesPoint = RightPitSamples.getPoint();


}