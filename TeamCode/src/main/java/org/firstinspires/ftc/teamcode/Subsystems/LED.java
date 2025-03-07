package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries

//this may cause errors in future look here first if something bad happens

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.LightBlinker;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Sample_Colors;


public class LED {
    RevBlinkinLedDriver blinkinLedDriver;
    ServoEx lightBlock;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    public static double offValue = 0;
    public static double redValue = 0.279;
    public static double yellowValue = 0.388;
    public static double blueValue = 0.611;
    public static double purpleValue = 0.720; //near end of color spectrum;
    public static double whiteValue = 1;


    //All LED objects

    //--------- ALL COMMANDS------------------
    public void init(HardwareMap hardwareMap){
        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lightBlock = hardwareMap.get(ServoEx.class,"lights");
        lightBlock.setPosition(purpleValue);
    }

    public void run_teleOp(Sample_Colors color){
        switch (color){
            case YELLOW:
                //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                lightBlock.setPosition(yellowValue);
                break;
            case RED:
                //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                lightBlock.setPosition(redValue);
                break;
            case BLUE:
                //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                lightBlock.setPosition(blueValue);
                break;
            case NONE:
                //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                lightBlock.setPosition(purpleValue);
                break;
        }
    }

    public void set(double colorValue){
        //blinkinLedDriver.setPattern(pattern);
        lightBlock.setPosition(colorValue);
    }


    //updates lighting in auto
    public void update(){}

    //turn off all lighting
    public void OFF(){
        lightBlock.setPosition(offValue);
    }

    public void getTelemetry(Telemetry telemetry){
        telemetry.addLine("----LED Lighting Data----");
        telemetry.addData("Lighting PWM: ", lightBlock.getPosition());
    }

}
