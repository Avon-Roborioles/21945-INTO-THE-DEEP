package org.firstinspires.ftc.teamcode.Utilities.Tests;

//import needed classes
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;

@Disabled
public class AutoMenuTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {

        //create follower & Input Gamepad
        GamepadEx driverOp = new GamepadEx(gamepad1);

        //initialize subsystems
        init_classes(driverOp);

        // call build paths method

        //show auto menu
        while(opModeInInit()) {
            runMenu();
            telemetry.update();
        }

        waitForStart();


    }
}
