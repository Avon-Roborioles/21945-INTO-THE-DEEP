package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


//robot subsystem for extendable arm
@Config
public class Arm {
    //motor objects & related variables
    Motor extensionMotor;
    Motor armMotor;
    public static final double GEAR_RATIO = 3.0; // Output 60 Teeth, Input 20 Teeth
    private static final  double ticks_in_degree = 700 / 180.0;

    //absolute positions ("final" means they can't change later in code)
    private final double groundPose = 0; //TODO
    private final double basket1Pose = 0; //TODO
    private final double basket2Pose = 0; //TODO
    private final double maxPose = 0; //TODO
    private double currentArmPose;
    private double currentEPose;
    private Arm_Poses armState;

    //TODO - Tune these PID variables
    PIDController pidController;
    public static final double p = 0;
    public static final double i = 0;
    public static final double d = 0;
    public static final double f = 0;
    public static int target = 0;

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
     * Default Arm Command to initialize motors & other variables
     * @param hardwareMap needed to access robot config
     */
    public void init(HardwareMap hardwareMap, GamepadEx gamepad){
        extensionMotor = new Motor(hardwareMap, "extensionMotor");
        armMotor = new Motor(hardwareMap, "armMotor");
        extensionMotor.setRunMode(Motor.RunMode.RawPower);
        armMotor.setRunMode(Motor.RunMode.RawPower);

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
        extensionMotor.set(-1);
        currentArmPose = 0;
    }


    /**
     * Comp-Ready Arm Command to initialize motors WITH PID Control
     * @param hardwareMap needed to access robot config
     */
    public void initPID(HardwareMap hardwareMap, GamepadEx gamepad){
        pidController = new PIDController(p,i,d);
        //telemetry
        extensionMotor = new Motor(hardwareMap, "extensionMotor");
        armMotor = new Motor(hardwareMap, "armMotor");
        extensionMotor.setRunMode(Motor.RunMode.RawPower);
        armMotor.setRunMode(Motor.RunMode.RawPower);
    }

    /**
     * Helps pull in the extensionArm & set the position to 0
     */
    public void setupEMotor(){
        if(d_up.getState()){
            extensionMotor.set(0);
            currentEPose = 0;
        } else {
            extensionMotor.set(-1);
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
//        currentArmPose = armMotor.getCurrentPosition();
//        currentEPose = armMotor.getCurrentPosition();

        //update leftY joystick reading
        leftY = driverOp.getLeftY();

        if(leftY > 0){
            armMotor.set(-0.6);
        } else if (leftY < 0){
            armMotor.set(0.6);
        } else {
            armMotor.set(-0.05); //0 passive hold
        }

        //arm extension control V1 - Greatly affects Arm Control & Can't use well
//        if(a_button.getState()) {
//            extensionMotor.set(0.2);
//        } else {
//            extensionMotor.set(-1);
//        }

        if(driverOp.gamepad.x){
            extensionMotor.set(-1);
        } else {
            extensionMotor.set(0);
        }

        if(driverOp.gamepad.b){
            extensionMotor.set(1);
        } else {
            extensionMotor.set(0);
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
        target = pose;
    }

    /**
     * update PID controller for arm in auto
     */
    public void update(){
        currentArmPose = armMotor.getCurrentPosition();
        //TODO Add extension PID Controller
        double pid = pidController.calculate(currentArmPose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree) * f);

        double power = pid + ff;

        armMotor.set(power);
    }


    public void getTelemetryBRIEF(Telemetry telemetry){
//        telemetry.addData("Arm Pose:", currentArmPose);
//        telemetry.addData("E Pose: ", currentEPose);
        telemetry.addData("D UP State: ", d_up.getState());
    }

    public void getTelemetryFULL(Telemetry telemetry){
        telemetry.addData("Arm Pose:", currentArmPose);
        telemetry.addData("E Pose: ", currentEPose);


    }
}
