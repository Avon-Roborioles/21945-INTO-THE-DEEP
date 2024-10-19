package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries

//this may cause errors in future look here first if something bad happens
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


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

    public enum COLORS{
        RED,
        BLUE,
        YELLOW,
        GREEN,
        PURPLE,
        GRAY,
        OFF
    }

    //All LED objects

    //----------TELEOP COMMANDS------------------
    public void init(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    public void setRainbow(){
        displayKind = DisplayKind.AUTO;

        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
//        display = telemetry.addData("Display Kind: ", displayKind.toString());
//        patternName = telemetry.addData("Pattern: ", pattern.toString());
//
//        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
//        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }


    //----------AUTO COMMANDS---------------------

    /**set all lighting to constant purple
     * - PURPLE, RED, YELLOW, BLUE, GREEN, GRAY
     */
    public void setConstantColor(COLORS color){
        switch(color){
            case PURPLE:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
            case RED:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            case YELLOW:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            case BLUE:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            case GREEN:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            case GRAY:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
            default:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }

    }

    //sets all lighting to breathing effect
    public void setBreathingMode(COLORS color){
        switch(color){
            case RED:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            case BLUE:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        }

    }

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
