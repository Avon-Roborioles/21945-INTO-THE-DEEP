package com.example.meepmeeptesting.Right_Autos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeptesting.SimPoseStorage;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Right_3Sample_Auto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(SimPoseStorage.RightStartPose)
                                .waitSeconds(8)

                            //setup
                                    //get first sample
                                    .lineToLinearHeading(SimPoseStorage.RightSample1)

                                    //give to human player
                                    .waitSeconds(.7)
                                    .lineToLinearHeading(SimPoseStorage.SampleDropoff)

                                    //get second sample
                                    .waitSeconds(.7)
                                    .lineToLinearHeading(SimPoseStorage.RightSample2)

                                    //drop second sample in corner
                                    .waitSeconds(.7)
                                    .lineToLinearHeading(SimPoseStorage.SampleDropoff)

                                    //pickup first specimen
                                    .waitSeconds(.7)
                                    .lineToLinearHeading(SimPoseStorage.SpecimenPickup)
                                    .waitSeconds(.1)
                                    .back(3)
                                    .waitSeconds(.2)
                                    .forward(3)

                            //score first specimen
                            .waitSeconds(.7)
                            .lineToLinearHeading(SimPoseStorage.SpecimenScore)

                                    //get third sample
                                    .waitSeconds(.7)
                                    .splineToLinearHeading(SimPoseStorage.RightSample3, SimPoseStorage.RightSample3.getHeading())

                                    //drop third sample in corner
                                    .waitSeconds(.7)
                                    .lineToLinearHeading(SimPoseStorage.SampleDropoff)

                                    //pickup second specimen
                                    .waitSeconds(.7)
                                    .lineToLinearHeading(SimPoseStorage.SpecimenPickup)
                                    .waitSeconds(.7)
                                    .back(3)
                                    .waitSeconds(.7)
                                    .forward(3)

                            //score second specimen
                            .waitSeconds(.7)
                            .lineToLinearHeading(SimPoseStorage.SpecimenScore)

                                    //pickup third specimen
                                    .waitSeconds(.7)
                                    .lineToLinearHeading(SimPoseStorage.SpecimenPickup)
                                    .waitSeconds(.7)
                                    .back(3)
                                    .waitSeconds(.1)
                                    .forward(3)


                            //score third specimen
                                .waitSeconds(.7)
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)

                            //park
                                .waitSeconds(.7)
                                .splineToLinearHeading(SimPoseStorage.RightPark, SimPoseStorage.RightPark.getHeading())
                                .waitSeconds(1)
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }
}