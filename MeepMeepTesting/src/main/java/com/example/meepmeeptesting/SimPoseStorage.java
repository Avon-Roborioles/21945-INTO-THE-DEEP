package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

//a copy of our main pose storage for our simulations (MeepMeep)
public class SimPoseStorage {

    //add currentPose for real poseStorage

   public static Pose2d LeftStartPose = new Pose2d(-35,-60, Math.toRadians(90));

   public  static Pose2d  RightStartPose = new Pose2d(11,-60, Math.toRadians(90));

   public static Pose2d  Sample1Field = new Pose2d(-35,-39,Math.toRadians(135));

   public static Pose2d  Sample2Field = new Pose2d(-46,-39,Math.toRadians(135));

   public static Pose2d  Sample3Field = new Pose2d(-52,-25,Math.toRadians(179));

   public static Pose2d BucketScore = new Pose2d(-49,-49,Math.toRadians(225));

   public static Pose2d PitSamples = new Pose2d(-28,-9,Math.toRadians(0));

   public static Pose2d LeftPark = new Pose2d(-29,-9.8, Math.toRadians(0));
}
