package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.*;

//this class is used by all auto programs to access subsystem controls & AutoMenu
public class AutoBase extends LinearOpMode {
    //subsystem objects - arm, intake, lift, etc
    protected org.firstinspires.ftc.teamcode.Subsystems.LED lighting = new LED();

    //auto pathing variables and arrays to loop through
    ToggleButtonReader d_up,d_down;

    public enum AutoPoses {
        LEFT,
        RIGHT
    }

    public AutoPoses AutoPose = AutoPoses.RIGHT; //default is right (samples)
    public int cycleCount = 1; //number of times to go to pit

    //blank runOpMode() method included only to keep LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {}

    public void init_classes(){

    }

    /**
    * used by new auto programs to init subsystems
     */
    public void init_classes(GamepadEx driverOp){
        d_up = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_UP
        );

        d_down = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_DOWN
        );
    }

    /**
     * Runs the Auto Menu Program to input Auto Pathing Selections
     */
    public void runMenu(){
        //menu code
        if(d_up.getState()){
            //startPose selection
            AutoPose = AutoPoses.RIGHT;
        } else {
            AutoPose = AutoPoses.LEFT;
        }

        if(d_down.wasJustPressed()){
            //cycle count selection
            if(cycleCount > 2){
                cycleCount = 1;
            } else {
                cycleCount++;
            }
        }


        //menu
        telemetry.addLine("Select the StartPose by Toggling the D-pad Up Button");
        telemetry.addData("Current StartPose Selected: ", AutoPose);
        telemetry.addLine(" ");
        telemetry.addLine("Select the # of Cycles by pressing the D-pad Down Button");
        telemetry.addData("Number of cycles: ", cycleCount);
        //telemetry.update();

        //updates
        d_up.readValue();
        d_down.readValue();

    }

    public AutoPoses getAutoPose(){
        return AutoPose;
    }


}
