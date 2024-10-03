package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//robot subsystem for active intake
public class Intake {
    //motor & servo objects

    //useful variables


    //enum commands for sample colors
    public enum Sample_Colors{
        YELLOW,
        BLUE,
        RED,
        NONE
    }


    //--------TELEOP COMMANDS------------
    public void init(HardwareMap hardwareMap){

    }

    //main command for teleOp code
    public void run_teleOp(GamepadEx driverOp){

    }

    //---------AUTO COMMANDS----------------
    //self explanatory
    public void run_wheels(Boolean isTrue){

    }


    //checks if a sample is collected by a valid color detection
    public Boolean Full(){
        boolean result = false;

        //color detection logic

        return result;
    }

    //returns color of sample, returns NONE if no color is detected
    public Sample_Colors get_sample_color(){
        Sample_Colors result = Sample_Colors.NONE;

        //logic

        return result;
    }

    public void getTelemetryBRIEF(Telemetry telemetry){

    }

    public void getTelemetryFULL(Telemetry telemetry){

    }




}
