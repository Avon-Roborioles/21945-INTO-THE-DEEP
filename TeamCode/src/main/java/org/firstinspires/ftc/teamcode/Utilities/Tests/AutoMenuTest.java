package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;

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

        //run AutoMenu in Init Loop
        while(opModeInInit()){
            runAutoMenu(driverOp);
        }

    }
}
