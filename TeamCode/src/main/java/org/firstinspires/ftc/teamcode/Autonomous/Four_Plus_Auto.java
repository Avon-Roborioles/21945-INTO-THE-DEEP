package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraires
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//TODO check if we're missing TrajectorySequence

@Disabled
//@Autonomous(name="4+ Cycle Auto", group = "Right Autos")
public class Four_Plus_Auto extends AutoBase {
    public void runOpMode() throws InterruptedException{
        //important variables
        GamepadEx driverOp = new GamepadEx(gamepad1);
        //initialize subsystems
        init_classes(driverOp);

        //TODO - vision.init_sample_detection(SAMPLE.COLOR - ALLIANCE COLOR)

        //Create trajectory sequences

        waitForStart();


    }

}
