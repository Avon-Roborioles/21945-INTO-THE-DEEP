package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//robot subsystem for lift slides, scoring rotation, & specimen intake
public class Lift {
    //motor & servo objects

    //absolute height positions
    double down = 0;
    double specimen = 0; //TODO
    double basket1 = 0; //TODO
    double basket2 = 0; //TODO
    double rung1 = 0; //TODO
    double rung2 = 0; //TODO

    //enum commands for lift positions
    public enum Lift_Poses{
        DOWN,
        SPECIMEN,
        BASKET1,
        BASKET2,
        RUNG1,
        RUNG2,
        //Add HANG1 & HANG2 if needed
        MAX
    }

}
