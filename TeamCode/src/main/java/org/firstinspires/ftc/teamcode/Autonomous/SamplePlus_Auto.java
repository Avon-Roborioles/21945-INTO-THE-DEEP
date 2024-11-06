package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraires
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//TODO check if we're missing TrajectorySequence

@Disabled
//@Autonomous(name="3 Plus Sample Auto", group = "Right Autos")
public class SamplePlus_Auto extends AutoBase {
    public void runOpMode() throws InterruptedException{
        //important variables

        //initialize subsystems
        init_classes();

        //TODO - vision.init_sample_detection(SAMPLE.COLOR - ALLIANCE COLOR)

        //Create trajectory sequences

        waitForStart();


    }

}
