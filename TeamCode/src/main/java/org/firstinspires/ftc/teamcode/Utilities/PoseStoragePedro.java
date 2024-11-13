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
    public static Pose CurrentPose = null;

    //Left Side Poses - Samples
    public static Pose LeftStartPose = new Pose(-35+72,-60+72, Math.toRadians(90)); //Done

    public static Pose LeftSample1 = new Pose(-41+72,-35+72, Math.toRadians(135)); //Done

    public static Pose LeftSample2 = new Pose(-50+72,-33+72, Math.toRadians(145)); //TODO MOVINGBack

    public static Pose LeftSample3 = new Pose(-64+72,-25+72,Math.toRadians(179)); //Done

    public static Pose LeftBucketScore = new Pose(-66+72,-56+72,Math.toRadians(225)); //Done

    public static Pose PitSamples = new Pose(-28+72,-26+72,Math.toRadians(0)); //TODO Hits cage

    public static Pose LeftCheckPoint = new Pose(-40+72,-43+72, Math.toRadians(225)); //TODO

    public static Pose LeftPark = new Pose(-30+72,72, Math.toRadians(0)); //Done



    //Right Side Poses - Specimen
    public static Pose RightStartPose = new Pose(11+72,-60+72, Math.toRadians(90)); //Done

    public static Pose RightSample1 = new Pose(43+72,-12+72,Math.toRadians(180)); //TODO

    public static Pose RightSample2 = new Pose(54+72,-12+72,Math.toRadians(180)); //TODO

    public static Pose RightSample3 = new Pose(61+72,-12+72,Math.toRadians(180)); //TODO

    public static Pose SampleDropoff = new Pose(54+72,-49+72,Math.toRadians(180)); //TODO

    public static Pose SpecimenPickup = new Pose(50+72,-57+72,Math.toRadians(90)); //TODO

    public static Pose SpecimenScore = new Pose(0+72,-36+72,Math.toRadians(-90)); //TODO

    public static Pose RightCheckPoint = new Pose(33+72,-40+72, Math.toRadians(180)); //TODO

    public static Pose RightPark = new Pose(-30+72,51, Math.toRadians(0)); //Done


    //useful points to hold during auto - allows us to keep our stance against aggressive bots
    public static Point BucketScorePoint = LeftBucketScore.getPoint();

    public static Point PitSamplesPoint = PitSamples.getPoint();



}