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
                                .waitSeconds(.1)
                                //test code here
                                //dropoff passive sample
                                .lineToLinearHeading(SimPoseStorage.SampleDropoff)
                                .waitSeconds(.1)

                                //pickup sample 1
                                .lineToLinearHeading(SimPoseStorage.RightSample1)
                                .waitSeconds(1)

                                //dropoff sample 1
                                .lineToLinearHeading(SimPoseStorage.SampleDropoff)
                                .waitSeconds(.1)

                                //pickup first specimen
                                .lineToLinearHeading(SimPoseStorage.SpecimenPickup)
                                .waitSeconds(.3)

                                //score first specimen
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)
                                .waitSeconds(.5)

                                //TODO make into a BezierCurve in PEdro-Pathing
                                //get sample 2
                                .lineToLinearHeading(SimPoseStorage.RightSample1)
                                .lineToLinearHeading(SimPoseStorage.RightSamplePushSpot)
                                .waitSeconds(.1)
                                .lineToLinearHeading(SimPoseStorage.RightSample2)
                                .waitSeconds(.1)


                                //dropoff sample 2
                                .lineToLinearHeading(SimPoseStorage.SamplePushDropoff)
                                .waitSeconds(.1)

                                //pickup second specimen
                                .lineToLinearHeading(SimPoseStorage.SpecimenPickup)
                                .waitSeconds(.3)

                                //score second specimen
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)
                                .waitSeconds(.5)

                                //get sample 3
                                .lineToLinearHeading(SimPoseStorage.RightSample1)
                                .lineToLinearHeading(SimPoseStorage.RightSamplePushSpot)
                                .waitSeconds(.1)
                                .lineToLinearHeading(SimPoseStorage.RightSample3)
                                .waitSeconds(.1)

                                //dropoff sample 3
                                .lineToLinearHeading(SimPoseStorage.SamplePushDropoff)
                                .waitSeconds(.1)

                                //pickup third specimen
                                .lineToLinearHeading(SimPoseStorage.SpecimenPickup)
                                .waitSeconds(.3)

                                //score third specimen
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)
                                .waitSeconds(.5)

                                //pickup fourth specimen
                                .lineToLinearHeading(SimPoseStorage.SpecimenPickup)
                                .waitSeconds(.3)

                                //score fourth specimen
                                .lineToLinearHeading(SimPoseStorage.SpecimenScore)
                                .waitSeconds(.5)

                                .waitSeconds(2000)
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }
}