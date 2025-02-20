package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries

//this may cause errors in future look here first if something bad happens

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Sample_Colors;


public class LED {
    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 10;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }



    //All LED objects

    //--------- ALL COMMANDS------------------
    public void init(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    }

    public void run_teleOp(Sample_Colors color){
        switch (color){
            case YELLOW:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case RED:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case BLUE:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            case NONE:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                break;
        }
    }

    public void set(RevBlinkinLedDriver.BlinkinPattern pattern){
        blinkinLedDriver.setPattern(pattern);

    }


    //updates lighting in auto
    public void update(){}

    //turn off all lighting
    public void OFF(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void getTelemetry(Telemetry telemetry){
        telemetry.addLine("----LED Lighting Data----");
        telemetry.addData("Lighting Connection: ", blinkinLedDriver.getConnectionInfo());
        telemetry.addData("Other Info: ", blinkinLedDriver.toString());
    }

}
