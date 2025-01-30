package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


//test program to tune Lift PID Controls & specimen intake
@Config
@TeleOp(name="Fresh Lift PID Control", group="Tests")
public class FreshPIDLiftControl extends LinearOpMode {

    public double previousTarget = 0;
    public static double liftTarget = 0;
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
        driverOp = new GamepadEx(gamepad1);
        rightY = driverOp.getRightY();
        time = new ElapsedTime();

        MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MotorEx liftMotor = new MotorEx(hardwareMap,"liftMotor");
        //reversing motor if needed
        liftMotor.setRunMode(Motor.RunMode.RawPower);


        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(liftMotor.getCurrentPosition(),0), new MotionState(liftTarget,0), maxVelocity,maxAcceleration);

        waitForStart();

    }
}
