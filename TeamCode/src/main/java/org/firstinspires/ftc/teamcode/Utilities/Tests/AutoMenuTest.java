package org.firstinspires.ftc.teamcode.Utilities.Tests;

//import needed classes
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;

@Autonomous
public class AutoMenuTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        //create follower & Input Gamepad
        GamepadEx driverOp;

        //important variables

        //initialize subsystems
        init_classes();
        driverOp = new GamepadEx(gamepad1);

        // call build paths method


        runMenu(driverOp);
        telemetry.update();


        waitForStart();


    }
}
