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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(SimPoseStorage.RightStartPose)
                                //score preload
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)
                                .waitSeconds(.01)

                                //move sample 1 in observation zone - TODO (make
                                .lineToLinearHeading(SimPoseStorage.RightSample1Start)
                                .splineToLinearHeading(SimPoseStorage.RightSample1,SimPoseStorage.RightSample1.getHeading())

                                //move sample 2 in observation zone

                                //move sample 3 in observation zone

                                //pickup specimen 1

                                //score specimen 1

                                //pickup specimen 2

                                //score specimen 2

                                //pickup specimen 3

                                //score specimen 4

                                //pickup specimen 5

                                //score specimen 5

                                //park

                                //--------------------------
                                .waitSeconds(2000)
                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }
}