package com.example.meepmeeptesting.Left_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeptesting.SimPoseStorage;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Left_3Plus_Sample_Auto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(SimPoseStorage.LeftStartPose)
                                .waitSeconds(1)

                                //score first sample
                                .lineToLinearHeading(SimPoseStorage.Sample1Field)
                                .waitSeconds(.7)
                                .lineToLinearHeading(SimPoseStorage.BucketScore)

                                .waitSeconds(.7)

                                //score second sample
                                .lineToLinearHeading(SimPoseStorage.Sample2Field)
                                .waitSeconds(.7)
                                .lineToLinearHeading(SimPoseStorage.BucketScore)

                                .waitSeconds(.7)

                                //sample third sample
                                .lineToLinearHeading(SimPoseStorage.Sample3Field)
                                .waitSeconds(.7)
                                .lineToLinearHeading(SimPoseStorage.BucketScore)

                                .waitSeconds(.7)

                                //pit cycle 1
                                .splineToLinearHeading(SimPoseStorage.PitSamples, SimPoseStorage.PitSamples.getHeading())
                                .waitSeconds(1)
                                .back(10)
                                .waitSeconds(.1)
                                .lineToLinearHeading(SimPoseStorage.BucketScore)

                                .waitSeconds(.7)

                                //pit cycle 2
                                .splineToLinearHeading(SimPoseStorage.PitSamples, SimPoseStorage.PitSamples.getHeading())
                                .waitSeconds(1)
                                .back(10)
                                .waitSeconds(.1)
                                .lineToLinearHeading(SimPoseStorage.BucketScore)
                                .waitSeconds(.7)

                                //park
                                .lineToLinearHeading(new Pose2d(SimPoseStorage.LeftPark.getX()-10, SimPoseStorage.LeftPark.getY(),SimPoseStorage.LeftPark.getHeading()))
                                .waitSeconds(.1)
                                .forward(10)
                                //park
//                                .lineToConstantHeading(new Vector2d(-40,-26))
//                                .waitSeconds(.01)
//                                .splineToLinearHeading(SimPoseStorage.LeftPark, SimPoseStorage.LeftPark.getHeading())
                                .waitSeconds(10)
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }
}