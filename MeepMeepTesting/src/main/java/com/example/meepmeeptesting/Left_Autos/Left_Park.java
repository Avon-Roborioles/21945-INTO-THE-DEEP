package com.example.meepmeeptesting.Left_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.example.meepmeeptesting.SimPoseStorage;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Left_Park {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(SimPoseStorage.LeftStartPose)
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(SimPoseStorage.LeftPark.getX()-10,SimPoseStorage.LeftPark.getY(), SimPoseStorage.LeftPark.getHeading()))
                                        .waitSeconds(.1)
                                        .lineToLinearHeading(SimPoseStorage.LeftPark)
                                        .waitSeconds(20000)
                                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }
}