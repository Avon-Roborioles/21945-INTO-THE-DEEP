package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


//test program to tune Lift PID Controls & specimen intake
@Config
@TeleOp(name="Fresh Lift PID Control", group="Tests")
public class FreshPIDLiftControl extends LinearOpMode {

    public double previousTarget = 0;
    public static double slidesTarget = 0;
    public static double intakeTarget = 0;
    public double slidesPower = 0;
    public static double kp = 0;
    public static double ka = 0;
    public static double maxVelocity = 0;
    public static double maxAcceleration = 0;
    public static double maxJerk = 0;
    public boolean motionComplete = true;
    private MotionProfile profile;

    double rightY;
    GamepadEx driverOp;
    ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
    }
}
