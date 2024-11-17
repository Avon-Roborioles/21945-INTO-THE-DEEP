package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import android.graphics.Color;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//robot subsystem for active intake
public class Intake {
    //motor & servo objects
    CRServo intakeServo;
    ColorSensor colorSensor;
    GamepadEx driverOp;

    //useful variables
    boolean intakeFull = true;
    Sample_Colors currentSampleColor;
    int intakePower = 0;

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
        intakeServo.setRunMode(Motor.RunMode.RawPower);
        //intakeServo.setInverted(true); //uncomment if positive power sucks in samples
        intakeServo.set(intakePower);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true); //turns on white LED for color detection
    }

    //main command for teleOp code
    public void run_teleOp(GamepadEx driverOp){

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
        return !(currentSampleColor == Sample_Colors.NONE);
    }

    //returns color of sample, returns NONE if no color is detected
    public Sample_Colors get_sample_color(){

        //logic

        return currentSampleColor;
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
