package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//robot subsystem for lift slides & specimen intake
public class Lift {
    //motor & servo objects

    //absolute height positions
    double downPose = 0;
    double specimenPose = 0; //TODO
    double basket1Pose = 0; //TODO
    double basket2Pose = 0; //TODO
    double rung1Pose = 0; //TODO
    double rung2Pose = 0; //TODO

    //other useful variables

    //enum commands for lift positions
    public enum Lift_Poses{
        GROUND,
        SPECIMEN,
        BASKET1,
        BASKET2,
        RUNG1,
        RUNG2,
        DRIVER_CONTROL,
        //Add HANG1 & HANG2 if needed
        MAX
    }

    //--------TELEOP COMMANDS-------------
    public void init(HardwareMap hardwareMap){

    }

    public void run_teleOp(GamepadEx driverOp){

    }

    //-------AUTO COMMANDS----------------

    //quick command to control lift
    public void set_lift(Lift_Poses pose){

    }

    //precise command to control lift
    public void set_lift(double pose){

    }

    //quick commands to control specimen claw
    public void open_specimen_claw(){

    }

    public void close_specimen_claw(){

    }

    public void getTelemetryBRIEF(Telemetry telemetry){

    }

    public void getTelemetryFULL(Telemetry telemetry){

    }

}
