package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

//a copy of our main pose storage for our simulations (MeepMeep)
public class SimPoseStorage {

    //add currentPose for real poseStorage

   //Left Side Poses
   public static Pose2d LeftStartPose = new Pose2d(-35,-60, Math.toRadians(90));

   public static Pose2d LeftSample1 = new Pose2d(-35,-39,Math.toRadians(135));

   public static Pose2d LeftSample2 = new Pose2d(-46,-39,Math.toRadians(135));

   public static Pose2d LeftSample3 = new Pose2d(-52,-25,Math.toRadians(179));

   public static Pose2d BucketScore = new Pose2d(-49,-49,Math.toRadians(225));

   public static Pose2d LeftPitSamples = new Pose2d(-28,-9,Math.toRadians(0));

   public static Pose2d LeftPark = new Pose2d(-29,-9.8, Math.toRadians(0));



   //Right Side Poses
   public static Pose2d RightStartPose = new Pose2d(11,-60, Math.toRadians(90));

   public static Pose2d RightSample1 = new Pose2d(35,-39,Math.toRadians(45));

   public static Pose2d RightSample2 = new Pose2d(46,-39,Math.toRadians(50));

   public static Pose2d RightSample3 = new Pose2d(52,-25,Math.toRadians(0));

   public static Pose2d RightPitSamples = new Pose2d(-28,-9,Math.toRadians(179));

   public static Pose2d SampleDropoff = new Pose2d(51,-40,Math.toRadians(-45));

   public static Pose2d SpecimenPickup = new Pose2d(50,-57,Math.toRadians(90));

   public static Pose2d SpecimenScore = new Pose2d(0,-33,Math.toRadians(-90));

   public static Pose2d PathCheckPoint = new Pose2d(33,-40, Math.toRadians(45));

   public static Pose2d RightPark = new Pose2d(29,-9.8, Math.toRadians(180-1e-6));
}
