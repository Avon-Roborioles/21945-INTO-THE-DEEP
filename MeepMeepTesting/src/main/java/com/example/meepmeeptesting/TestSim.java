package com.example.meepmeeptesting;

//import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class TestSim {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(SimPoseStorage.RightStartPose)
                                //score preload
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)
                                .waitSeconds(.01)

                                //move sample 1 in observation zone - TODO pathChain with tangential heading (looks cool lol)
                                .lineToLinearHeading(SimPoseStorage.RightSample1Start)
                                .lineToLinearHeading(SimPoseStorage.RightSample1Curve)
                                .lineToLinearHeading(SimPoseStorage.RightSample1)
                                .lineToLinearHeading(SimPoseStorage.RightSample1Push)

                                //move sample 2 in observation zone - TODO pathChain with constant heading
                                .lineToLinearHeading(SimPoseStorage.RightSample1)
                                .lineToLinearHeading(SimPoseStorage.RightSample2)
                                .lineToLinearHeading(SimPoseStorage.RightSample2Push)

                                //move sample 3 in observation zone - TODO pathChain with constant heading
                                .lineToLinearHeading(SimPoseStorage.RightSample2)
                                .lineToLinearHeading(SimPoseStorage.RightSample3)
                                .lineToLinearHeading(SimPoseStorage.RightSample3Push)

                                //pickup specimen 1
                                .lineToLinearHeading(SimPoseStorage.SpecimenPickup)

                                //score specimen 1
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)

                                //pickup specimen 2
                                .lineToLinearHeading(SimPoseStorage.SpecimenPickup)

                                //score specimen 2
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)

                                //pickup specimen 3
                                .lineToLinearHeading(SimPoseStorage.SpecimenPickup)

                                //score specimen 3
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)

//                                //pickup specimen 4
//                                .lineToLinearHeading(SimPoseStorage.SpecimenPickup)
//
//                                //score specimen 4
//                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)

                                //park
                                .lineToLinearHeading(SimPoseStorage.RightPark)

                                //--------------------------
                                .waitSeconds(2000)
                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }
}