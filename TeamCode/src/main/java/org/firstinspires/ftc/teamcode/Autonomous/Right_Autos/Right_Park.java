package org.firstinspires.ftc.teamcode.Autonomous.Right_Autos;

//import needed libraires
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Utilities.RoadRunner.MecanumDrive;
//TODO check if we're missing TrajectorySequence

@Autonomous(name="Right Park", group = "Right Autos")
public class Right_Park extends AutoBase {
    public void runOpMode() throws InterruptedException{
        //important variables

        //initialize subsystems
        init_classes();

        //TODO - vision.init_sample_detection(SAMPLE.COLOR - ALLIANCE COLOR)

        //Create trajectory sequences

        waitForStart();


    }

}
