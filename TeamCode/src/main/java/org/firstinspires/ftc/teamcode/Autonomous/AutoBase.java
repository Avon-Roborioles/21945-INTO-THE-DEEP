package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.*;

//this class is used by all auto programs to access subsystem controls & AutoMenu
public class AutoBase extends LinearOpMode {
    //subsystem objects - arm, intake, lift, etc
    protected org.firstinspires.ftc.teamcode.Subsystems.LED lighting = new LED();

    //blank runOpMode() method included only to keep LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {}


    /**
    * used by auto programs to init subsystems
     */
    public void init_classes(){

    }

    /**
     * Runs the AutoMenu Program to input Auto Pathing Selections
     */
    public void runAutoMenu(GamepadEx driverOp){

    }


}
