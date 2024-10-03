package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.controller.PIDFController;

//robot subsystem for extendable arm
public class Arm {
    //motor & servo objects

    //absolute positions ("final" means they can't change later in code)
    private final double groundPose = 0; //TODO
    private final double basket1Pose = 0; //TODO
    private final double basket2Pose = 0; //TODO
    private final double maxPose = 0; //TODO
    private double currentPose = 0;
    private Arm_Poses armState = null;

    //TODO - PID variables
    double target = 0;
    private final double p = 0;
    private final double i = 0;
    private final double d = 0;
    private final double f = 0;

    //useful variables

    //enum commands for arm positions
    public enum Arm_Poses {
        GROUND,
        BASKET1,
        BASKET2,
        DRIVER_CONTROL,
        MAX
    }

    //--------TELEOP COMMANDS---------
    public void init(HardwareMap hardwareMap){

    }

    public void run_teleOp(GamepadEx driverOp){

    }

    //--------AUTO COMMANDS------------

    //quick command to control arm
    public void set_arm(Arm_Poses pose){

    }

    //precise command to control arm
    public void set_arm(int pose){

    }

    //update PID controller for arm in auto
    public void update(){
        
    }


    public void getTelemetryBRIEF(Telemetry telemetry){
        telemetry.addData("Arm Pose:", currentPose);
    }

    public void getTelemetryFULL(Telemetry telemetry){
        telemetry.addData("Arm Pose:", currentPose);
        telemetry.addData("Arm STATE:", armState);


    }
}
