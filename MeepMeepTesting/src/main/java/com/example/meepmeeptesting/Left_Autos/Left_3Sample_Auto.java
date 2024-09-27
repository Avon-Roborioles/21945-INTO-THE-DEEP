package com.example.meepmeeptesting.Left_Autos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeptesting.SimPoseStorage;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Left_3Sample_Auto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(SimPoseStorage.LeftStartPose)
                                .waitSeconds(1)

                                //score first sample
                                .lineToLinearHeading(SimPoseStorage.LeftSample1)
                                .waitSeconds(.7)
                                .lineToLinearHeading(SimPoseStorage.BucketScore)

                                .waitSeconds(.7)

                                //score second sample
                                .lineToLinearHeading(SimPoseStorage.LeftSample2)
                                .waitSeconds(.7)
                                .lineToLinearHeading(SimPoseStorage.BucketScore)

                                .waitSeconds(.7)

                                //sample third sample
                                .lineToLinearHeading(SimPoseStorage.LeftSample3)
                                .waitSeconds(.7)
                                .lineToLinearHeading(SimPoseStorage.BucketScore)

                                .waitSeconds(.7)

                                //park
                                .lineToConstantHeading(new Vector2d(-40,-26))
                                .waitSeconds(.01)
                                .splineToLinearHeading(SimPoseStorage.LeftPark, SimPoseStorage.LeftPark.getHeading())

                                .waitSeconds(10)
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }
}