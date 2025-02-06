package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//robot subsystem for active intake
public class Intake {
    //motor & servo objects
    CRServo intakeServo;
    NormalizedColorSensor colorSensor;
    GamepadEx driverOp;

    //useful variables
    boolean intakeFull = true;
    Sample_Colors currentSampleColor;
    double intakePower = 0;
    double intakeDistance = 0; //cm
    boolean autoMode = false;
    boolean pickup = true;
    ElapsedTime time;

    //Raw Color Readings with (Gain 2):
    //Yellow - 0.031 red, 0.042 green, 0.013 blue
    //Red - 0.147 red, 0.087 green, 0.053 blue
    //Blue - 0.008 red, 0.015 green, 0.03 blue


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
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(2);
        intakeServo.setRunMode(Motor.RunMode.RawPower);
        intakeServo.set(intakePower);
        time = new ElapsedTime();

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

        if(!autoMode){
            if(driverOp.gamepad.left_trigger > 0){
                intakePower = 1;
            } else if(driverOp.gamepad.right_trigger > 0){
                intakePower = -1;
            } else {
                intakePower = 0;
            }


          if(right_bumper.wasJustPressed()){
                intakePower = -1;
                autoMode = true;
            }

        } else {
            //logic to auto pickup/drop + get out of auto mode

            //full
            if(isFull()){
                intakePower = 0;
                autoMode = false;
            }
            //left trigger
            if(driverOp.gamepad.left_trigger > 0){
                intakePower = 1;
                autoMode = false;
            } else if(driverOp.gamepad.right_trigger > 0){
                //right trigger
                intakePower = -1;
                autoMode = false;

            }

        }

        intakeServo.set(intakePower);

    }

    //---------AUTO COMMANDS----------------
    public void run(boolean forward){
        if(forward){
            intakePower = -1;
        } else {
            intakePower = 1;
        }
        intakeServo.set(intakePower);
    }

    //self explanatory
    public void pickup(){
        autoMode = true;
        intakePower = -0.85;
        pickup = true;
    }

    public void drop(){
        autoMode = true;
        intakePower = -1;
        pickup = false;
        time.reset();
    }

    public void stop(){
        intakePower = 0;
    }

    public void update(){ //auto
        if(autoMode){
            if(pickup){
                if(isFull()){
                    autoMode = false;
                }
            } else {
                if(!isFull() && time.seconds() > 1.5){
                    autoMode = false;
                }
            }
        } else {
            intakePower = 0;
        }
        intakeServo.set(intakePower);
    }


    //checks if a sample is collected by a valid color detection
    public boolean isFull(){
        intakeDistance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        return intakeDistance < 6.5; //5
    }

    //returns color of sample, returns NONE if no color is detected
    public Sample_Colors get_sample_color(){
        //Raw Color Readings with (Gain 2):
        //Yellow - 0.031 red, 0.042 green, 0.013 blue
        //Red - 0.147 red, 0.087 green, 0.053 blue
        //Blue - 0.008 red, 0.015 green, 0.03 blue
        //logic
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double redValue = colors.red;
        double greenValue = colors.green;
        double blueValue = colors.blue;

        if(isFull()){
            if(redValue > greenValue && redValue > blueValue){
                currentSampleColor = Sample_Colors.RED;
            } else if(blueValue > redValue){
                currentSampleColor = Sample_Colors.BLUE;
            } else if(greenValue > redValue){
                currentSampleColor = Sample_Colors.YELLOW;
            } else {
                currentSampleColor = Sample_Colors.NONE;
            }
        } else {
            currentSampleColor = Sample_Colors.NONE;
        }
        return currentSampleColor;
    }


    public void getTelemetryBRIEF(Telemetry telemetry){
        telemetry.addLine("-----Intake Control Data-----");
        telemetry.addData("Current Sample Color: ", currentSampleColor);
        telemetry.addData("Intake Full?: ", isFull());
        telemetry.addData("Intake Power: ", intakePower);
        telemetry.addData("Auto Mode?: ", autoMode);
    }

    public void getTelemetryFULL(Telemetry telemetry){
        telemetry.addLine("-----Intake Control Data-----");
        telemetry.addData("Current Sample Color: ", get_sample_color());
        telemetry.addData("Intake Full?: ", isFull());
        telemetry.addData("Intake Power: ", intakePower);
        telemetry.addData("Intake Auto Mode: ", autoMode);
        telemetry.addData("Intake Drop Timer: ", time.seconds());
    }
}
