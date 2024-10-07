package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Driver_Feedback {

    boolean secondHalf = false;                 // Use to prevent multiple half-time warning rumbles.

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    ElapsedTime runtime = new ElapsedTime();    // Use to determine when end game is starting.

    final double HALF_TIME = 60.0;              // Wait this many seconds before rumble-alert for half-time.
    final double TRIGGER_THRESHOLD  = 0.75;     // Squeeze more than 3/4 to get rumble.

    //starts the end-game timer to alert drivers

}
