package org.firstinspires.ftc.teamcode.Autonomous;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.*;

//this class is used by all auto programs to access subsystem controls & AutoMenu
public class AutoBase extends LinearOpMode {
    //subsystem objects - arm, intake, lift, etc
    protected org.firstinspires.ftc.teamcode.Subsystems.LED lighting = new LED();
    protected org.firstinspires.ftc.teamcode.Subsystems.AutoMenu menu = new AutoMenu();

    //important variables
    String[] menuOptions = {"Path 1", "Path 2", "Path 3"};

    //blank runOpMode() method included only to keep LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {}


    /**
    * used by auto programs to init subsystems
     */
    public void init_classes(){
        menu.init(telemetry, menuOptions);
    }

    /**
     * Runs the AutoMenu Program to input Auto Pathing Selections
     */
    public void runMenu(GamepadEx driverOp){
        //controls for autoMenu
        if(driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            menu.navigateUp();
        } else if(driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            menu.navigateDown();
        }

        //shows options along with selected choice
        menu.showMenu();
        telemetry.addLine("--------------");
        telemetry.addData("Selected Choice: ", menu.getSelectedOption());
        //telemetry.update();
    }


}
