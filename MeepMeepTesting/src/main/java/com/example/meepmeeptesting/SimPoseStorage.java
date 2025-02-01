package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

//a copy of our main pose storage for our simulations (MeepMeep)
public class SimPoseStorage {

    //add currentPose for real poseStorage

   //Left Side Poses
   public static Pose2d LeftStartPose = new Pose2d(37-72,12-72, Math.toRadians(90));

   public static Pose2d LeftSample1 = new Pose2d(37-72,49.2-72,Math.toRadians(180-1e-6));

   public static Pose2d LeftSample2 = new Pose2d(25.7-72,49.2-72,Math.toRadians(180-1e-6));

   public static Pose2d LeftSample3 = new Pose2d(-52,-25,Math.toRadians(180-1e-6));

   public static Pose2d BucketScore = new Pose2d(-49,-49,Math.toRadians(225));

   public static Pose2d LeftPitSamples = new Pose2d(-28,-9,Math.toRadians(0));

   public static Pose2d LeftPark = new Pose2d(-29,-9.8, Math.toRadians(0));

   public static Pose2d LeftCheckPoint = new Pose2d(-40,-43, Math.toRadians(225));




   //Right Side Poses
   public static Pose2d RightStartPose = new Pose2d(11,-60, Math.toRadians(90)); //TODO

   public static Pose2d RightSample1 = new Pose2d(45,-12,Math.toRadians(180));//TODO

   public static Pose2d RightSample1Start = new Pose2d(35,-35,Math.toRadians(180)); //TODO

   public static Pose2d RightSample1Curve = new Pose2d(38,-12,Math.toRadians(180)); //TODO NEW

   public static Pose2d RightSample1Push = new Pose2d(45,-57,Math.toRadians(180));//TODO


   public static Pose2d RightSamplePushSpot = new Pose2d(43, -12, Math.toRadians(180));

   public static Pose2d RightSample2 = new Pose2d(54,-12,Math.toRadians(180));//TODO

   public static Pose2d RightSample2Push = new Pose2d(54,-57,Math.toRadians(180));//TODO

   public static Pose2d RightSample3 = new Pose2d(61,-12,Math.toRadians(180)); //TODO

   public static Pose2d RightSample3Push = new Pose2d(61,-57,Math.toRadians(180));//TODO

   public static Pose2d SampleDropoff = new Pose2d(54,-49,Math.toRadians(-45)); //TODO

   public static Pose2d SamplePushDropoff -35+72,= new Pose2d(59, -49, Math.toRadians(180));

   public static Pose2d SpecimenPickup = new Pose2d(40,-57,Math.toRadians(0)); //TODO

   public static Pose2d SpecimenScore = new Pose2d(0,-35,Math.toRadians(180-1e-6)); //TODO

   public static Pose2d RightPark = new Pose2d(50,-54, Math.toRadians(180-1e-6)); //TODO
}
