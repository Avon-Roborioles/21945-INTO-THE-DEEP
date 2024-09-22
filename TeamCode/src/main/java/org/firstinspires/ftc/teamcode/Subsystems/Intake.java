package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//robot subsystem for active intake, horizontal slides, & transfer rotation(dump w/ servo)
public class Intake {
    //motor & servo objects


    //enum commands for slides positions
    public enum Slides_Poses{
        RETRACT, //fully in bot
        STANDARD, //typical extension in pit
        MAX //max extension
    }

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

    //quick command to control slides
    public void set_slides(Slides_Poses pose){

    }

    //precise command to control slides
    public void set_slides(double pose){

    }

    //moves intake up or down to transfer
    public void transfer(Boolean isTrue){

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
