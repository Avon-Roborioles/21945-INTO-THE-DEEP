package org.firstinspires.ftc.teamcode.Autonomous.Left_Autos;

//import needed libraires
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Utilities.RoadRunner.MecanumDrive;
//TODO check if we're missing TrajectorySequence

@Disabled
//@Autonomous(name="3 Sample Auto", group = "Left Autos")
public class Left_3SAMPLE_Auto extends AutoBase {
    public void runOpMode() throws InterruptedException{
        //important variables

        //initialize subsystems
        init_classes();

        //TODO - vision.init_sample_detection(SAMPLE.COLOR - YELLOW)

        //Create trajectory sequences

        waitForStart();


    }

}
