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
    Motor extensionMotor;
    Motor armMotor;
    static final double GEAR_RATIO = 10.0; //TODO record actual gear and store here

    //absolute positions ("final" means they can't change later in code)
    private final double groundPose = 0; //TODO
    private final double basket1Pose = 0; //TODO
    private final double basket2Pose = 0; //TODO
    private final double maxPose = 0; //TODO
    private double currentPose;
    private Arm_Poses armState;

    double targetReach; //TODO - a crucial value calculated & used to position arm + tolerence (length of intake)

    //TODO - PID variables
    double target = 0;
    private final double p = 0;
    private final double i = 0;
    private final double d = 0;
    private final double f = 0;

    //control variables
    GamepadEx driverOp;
    ToggleButtonReader a_button;
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
        extensionMotor = new Motor(hardwareMap, "extensionMotor");
        armMotor = new Motor(hardwareMap, "armMotor");
        extensionMotor.setRunMode(Motor.RunMode.RawPower);
        armMotor.setRunMode(Motor.RunMode.RawPower);

        //gamepad variables
        driverOp = gamepad;
        a_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );
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
//        double leftY = gamepad2.left_stick_y;
//        float rightTrigger = gamepad2.right_trigger;
//
//        //activate Hanging Mode
//        if(d_down.wasJustPressed()){
//            hangDefault = true;
//        }
//
//        if(leftY > 0 || leftY < 0) {
//            hangDefault = false;
//            if (rightMotorEx.getCurrentPosition() < 1600) {
//                if (leftY < 0) {
//                    leftMotorEx.setPower(-0.5);
//                    rightMotorEx.setPower(0.5);
//                } else if (leftY > 0) {
//                    leftMotorEx.setPower(0.3);
//                    rightMotorEx.setPower(-0.3);
//                }
//            } else {
//                if (leftY > 0) {
//                    leftMotorEx.setPower(0.5);
//                    rightMotorEx.setPower(-0.5);
//                } else if (leftY < 0) {
//                    leftMotorEx.setPower(-0.3);
//                    rightMotorEx.setPower(0.3);
//                }
//            }
//        } else if(!hangDefault){
//            if(rightMotorEx.getCurrentPosition() < 1600) {
//                leftMotorEx.setPower(-0.04); //small bit of power for brakes
//                rightMotorEx.setPower(0.04);
//            } else {
//                leftMotorEx.setPower(0.09); //small bit of power for brakes - 0.09
//                rightMotorEx.setPower(-0.09);
//            }
//        }
        //main arm control
        leftY = driverOp.getLeftY();

        if(leftY > 0){
            armMotor.set(0.5);
        } else if (leftY < 0){
            armMotor.set(-0.5);
        } else {
            armMotor.set(0);
        }

        //arm extension control
        if(a_button.wasJustPressed()){
            extensionMotor.set(1);
        } else {
            extensionMotor.set(0);
        }

        a_button.readValue(); //upddate a_button toggle
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
        telemetry.addData("Arm Pose:", currentPose);
    }

    public void getTelemetryFULL(Telemetry telemetry){
        telemetry.addData("Arm Pose:", currentPose);
        telemetry.addData("Arm STATE:", armState);


    }
}
