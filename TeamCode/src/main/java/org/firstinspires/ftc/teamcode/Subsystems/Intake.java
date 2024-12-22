package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//robot subsystem for active intake
public class Intake {
    //motor & servo objects
    CRServo intakeServo;
    ColorSensor colorSensor;
    TouchSensor touchSensor;
    GamepadEx driverOp;

    //useful variables
    boolean intakeFull = true;
    Sample_Colors currentSampleColor;
    int intakePower = 0;

    ToggleButtonReader y_button, a_button, x_button, b_button; //modes
    ToggleButtonReader d_up, d_down, d_left, d_right; //height toggles
    ToggleButtonReader left_bumper, right_bumper; //intake control w/ sample detection

    //enum commands for sample colors
    public enum Sample_Colors{
        YELLOW,
        BLUE,
        RED,
        NONE
    }

    //--------TELEOP COMMANDS------------
    public void init(HardwareMap hardwareMap, GamepadEx gamepad){
        driverOp = gamepad;

        intakeServo = new CRServo(hardwareMap, "intake");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        intakeServo.setRunMode(Motor.RunMode.RawPower);
        intakeServo.set(intakePower);

        //---initialize toggles & buttons---
        d_up = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_UP
        );
        d_down = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_DOWN
        );
        d_left = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_LEFT
        );
        d_right = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_RIGHT
        );

        y_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.Y
        );
        a_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );
        x_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );
        b_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.B
        );
        left_bumper = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.LEFT_BUMPER
        );
        right_bumper = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_BUMPER
        );

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true); //turns on white LED for color detection
    }

    /**
     * Init Method to Simulate Scoring with Touch Sensor
     */
    public void initTest(HardwareMap hardwareMap, GamepadEx gamepad){
        driverOp = gamepad;
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

    }

    public boolean isTouched(){
        return touchSensor.isPressed();
    }

    private void updateToggles(){
        d_up.readValue();
        d_down.readValue();
        d_left.readValue();
        d_right.readValue();
        y_button.readValue();
        a_button.readValue();
        x_button.readValue();
        b_button.readValue();
        left_bumper.readValue();
        right_bumper.readValue();
    }

    //main command for teleOp code
    public void run_teleOp(){
        updateToggles();

        //manual trigger control
        if(driverOp.gamepad.left_trigger > 0){
            intakePower = 1;
            intakeServo.set(intakePower);
        } else if (driverOp.gamepad.right_trigger > 0){
            intakePower = -1;
            intakeServo.set(intakePower);
        } else {
            intakePower = 0;
            intakeServo.set(intakePower);
        }

        //TODO remove later
//        if(left_bumper.wasJustPressed()){
//            intakeSwivel.setPosition(0);
//        } else if(right_bumper.wasJustPressed()){
//            intakeSwivel.setPosition(1);
//        } else if(a_button.wasJustPressed()){
//            intakeSwivel.setPosition(0.4);
//        }

    }

    //---------AUTO COMMANDS----------------
    //self explanatory
    public void pickup(){
        //intakeServo.set(-1);
        intakePower = -1;
        //color sensor logic - turn off when sample is detected
    }

    public void drop(){
        //intakeServo.set(1);
        intakePower = 1;
        //color sensor logic - turn off when sample is not detected
    }

    public void stop(){
        //intakeServo.stop();
        intakePower = 0;
    }

    public void update(){
        intakeServo.set(intakePower);
    }


    //checks if a sample is collected by a valid color detection
    public Boolean isFull(){
        double intakeDistance =
    }

    //returns color of sample, returns NONE if no color is detected
    public Sample_Colors get_sample_color(){

        //logic

        return currentSampleColor;
    }

    public void getTelemetryTest(Telemetry telemetry){
        telemetry.addData("Touch Sensor Status: ", touchSensor.isPressed());
    }

    public void getTelemetryBRIEF(Telemetry telemetry){
        telemetry.addLine("-----Intake Control Data-----");
        telemetry.addData("Current Sample Color: ", currentSampleColor);
        telemetry.addData("Intake Full?: ", intakeFull);
        telemetry.addData("Intake Power: ", intakePower);
    }

    public void getTelemetryFULL(Telemetry telemetry){
        telemetry.addLine("-----Intake Control Data-----");
        telemetry.addData("Current Sample Color: ", currentSampleColor);
        telemetry.addData("Intake Full?: ", intakeFull);
        telemetry.addData("Intake Power: ", intakePower);
    }
}
