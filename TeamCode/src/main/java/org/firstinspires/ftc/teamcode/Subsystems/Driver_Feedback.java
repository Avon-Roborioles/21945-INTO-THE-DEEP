package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Driver_Feedback {
    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.

    final double HALF_TIME = 60.0;              // Wait this many seconds before rumble-alert for half-time.
    final double TRIGGER_THRESHOLD  = 0.75;     // Squeeze more than 3/4 to get rumble.
    ElapsedTime teleOpTimer;                    // used to determine when endgame has started
    boolean Driver1Alert = false;
    boolean Driver2Alert = false;

    //starts the end-game timer to alert drivers
    public void init(){
        teleOpTimer = new ElapsedTime(90);
    }

    //start teleOp timer
    public void startTimer(){
        teleOpTimer.startTime();

    }

    public void alert_driver(GamepadEx driverOp){
        driverOp.gamepad.rumble(500);
    }

    public void alert_drivers(GamepadEx driver1Op, GamepadEx driver2Op){}

    public void getTelemetry(Telemetry telemetry){
        telemetry.addLine("----Haptic FeedBack Data----");
        telemetry.addData("Time Left: ", teleOpTimer.toString());

    }
}
