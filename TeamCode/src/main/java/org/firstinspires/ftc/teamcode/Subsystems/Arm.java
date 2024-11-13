package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.controller.PIDFController;

//robot subsystem for extendable arm
public class Arm {
    //motor objects & related variables
    Motor extentionMotor;
    Motor armMotor;
    static final double GEAR_RATIO = 10.0; //TODO record actual gear and store here

    //absolute positions ("final" means they can't change later in code)
    private final double groundPose = 0; //TODO
    private final double basket1Pose = 0; //TODO
    private final double basket2Pose = 0; //TODO
    private final double maxPose = 0; //TODO
    private double currentArmPose;
    private double currentEPose;
    private Arm_Poses armState;

    //TODO - PID variables
    double target = 0;
    private final double p = 0;
    private final double i = 0;
    private final double d = 0;
    private final double f = 0;

    //control variables
    GamepadEx driverOp;
    ToggleButtonReader a_button, d_up;
    double leftY;

    //enum commands for arm positions
    public enum Arm_Poses {
        GROUND,
        BASKET1,
        BASKET2,
        SPECIMEN_PICKUP,
        RUNG1,
        RUNG2,
        DRIVER_CONTROL,
        MAX
    }

    //--------TELEOP COMMANDS---------

    /**
     * Arm Command to initialize motors & other variables
     * @param hardwareMap needed to access robot config
     */
    public void init(HardwareMap hardwareMap, GamepadEx gamepad){
        extentionMotor = new Motor(hardwareMap, "extensionMotor");
        armMotor = new Motor(hardwareMap, "armMotor");
        extentionMotor.setRunMode(Motor.RunMode.PositionControl);
        armMotor.setRunMode(Motor.RunMode.PositionControl);

        //gamepad variables
        driverOp = gamepad;

        //extensionMotor toggle
        a_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );

        //button to set extensionMotor to 0
        d_up = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_UP
        );

        //start running EMotor & set ArmPose to 0
        extentionMotor.set(-1);
        currentArmPose = 0;
    }

    /**
     * Helps pull in the extensionArm & set the position to 0
     */
    public void setupEMotor(){
        if(d_up.wasJustPressed()){
            extentionMotor.set(0);
            currentEPose = 0;
        }

        d_up.readValue();
    }

    /**
     * competition-rated method using a PID Controller for precise movement
     * @param driverOp needed to control arm
     */
    public void run_teleOp(GamepadEx driverOp){

    }

    /**
     * testing-rated method using raw power values for movement
     */
    public void run_teleOpBASIC(){
        currentArmPose = armMotor.getCurrentPosition();
        currentEPose = armMotor.getCurrentPosition();

        //update leftY joystick reading
        leftY = driverOp.getLeftY();

        if(leftY > 0){
            armMotor.set(-0.5);
        } else if (leftY < 0){
            armMotor.set(0.5);
        } else {
            armMotor.set(0);
        }

        //arm extension control
        if(a_button.getState()) {
            extentionMotor.set(0.2);
        } else {
            extentionMotor.set(-1);
        }


        a_button.readValue(); //update a_button toggle
    }

    //--------AUTO COMMANDS------------

    /**
     * main command to control arm
     * @param pose enum Arm State
     */
    public void set_arm(Arm_Poses pose){

    }

    /**
     * precise command to control arm
     * @param pose double value of arm position
     */
    public void set_arm(int pose){

    }

    /**
     * update PID controller for arm in auto
     */
    public void update(){
        
    }


    public void getTelemetryBRIEF(Telemetry telemetry){
        telemetry.addData("Arm Pose:", currentArmPose);
        telemetry.addData("E Pose: ", currentEPose);
    }

    public void getTelemetryFULL(Telemetry telemetry){
        telemetry.addData("Arm Pose:", currentArmPose);
        telemetry.addData("E Pose: ", currentEPose);
        telemetry.addData("Arm STATE:", armState);


    }
}
