package org.firstinspires.ftc.teamcode.Subsystems;

//imported libraries needed
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Driver_Feedback {
    //useful objects & variables
    ElapsedTime endgameTimer;
    boolean Driver1Alert = false;
    boolean Driver2Alert = false;

    /**
     * creates the end-game timer to alert drivers
     */
    public void init(){
        endgameTimer = new ElapsedTime(90);
    }

   /**
    * starts the Endgame Timer
    */
    public void startTimer(){
        endgameTimer.startTime();

    }

    /**
     *alerts the specifed driver with a percise amount of strength; mainly used for subsystem aerts
     */
    public void alert_driver(GamepadEx driverOp, int strength){
        Gamepad.RumbleEffect strength40;
        Gamepad.RumbleEffect strength70;
        Gamepad.RumbleEffect strength100;

        strength40 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.4, 0.0, 500)  //  Rumble right motor 80% for 500 mSec
                .build();
        strength70 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.7, 0.0, 500)  //  Rumble right motor 90% for 500 mSec
                .build();
        strength100 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 500)  //  Rumble right motor 100% for 500 mSec
                .build();

        if (strength == 40) {
            driverOp.gamepad.runRumbleEffect(strength40);
        } else if (strength == 70) {
            driverOp.gamepad.runRumbleEffect(strength70);
        } else if (strength == 100) {
            driverOp.gamepad.runRumbleEffect(strength100);
        } else {
            driverOp.gamepad.rumble(500);
        }
    }


    /**
     * alerts the specifed driver with a half-second (500ms) rumble
     */
    public void alert_driver(GamepadEx driverOp){
        driverOp.gamepad.rumble(500);
    }


    /**
     * alerts both driver with 3 rumbles on both gamepads
     */
    public void alert_drivers(GamepadEx driver1Op, GamepadEx driver2Op){
        driver1Op.gamepad.rumbleBlips(3);
        driver2Op.gamepad.rumbleBlips(3);
    }


    /**
    @return Time left from the endgame timer since started by the primary driver
     */
    public String getTimer(){
        return endgameTimer.toString();
    }
}
