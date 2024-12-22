package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="Fresh Arm PID Control", group="Tests")
public class FreshPIDArmControl extends LinearOpMode {

    public double previousTarget = 0;
    public static double armTarget = 0;
    public double armPower = 0;
    public static double kp = 0.05;
    public static double ki = 0;
    public static double kd = 0;
    public static double maxVelocity = 60;
    public static double maxAcceleration = 60;
    public static double maximumIntegralSum = 0;
    public static double stability = 0;
    public static double lowPassGain = 0;
    public static double maxJerk = 0;
    public boolean motionComplete = true;
    double startTime = System.currentTimeMillis();
    private MotionProfile profile;
    private TrapezoidProfile newProfile;
    public MotionState currentState;
    VelocityConstraint velocityConstraint;
    AccelerationConstraint accelerationConstraint;

    double leftY;
    GamepadEx driverOp;
    ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad2);
        leftY = driverOp.getLeftY();
        time = new ElapsedTime();

        //init PID
        Motor armMotor = new Motor(hardwareMap,"armMotor");
        MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor.setInverted(true);
        armMotor.encoder.setDirection(Motor.Direction.REVERSE);
        armMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.stopAndResetEncoder();
        armMotor.setRunMode(Motor.RunMode.RawPower);
        PIDCoefficientsEx armCoefficients = new PIDCoefficientsEx(kp,ki,kd,maximumIntegralSum,stability,lowPassGain);
        PIDEx armController = new PIDEx(armCoefficients);

        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(armMotor.getCurrentPosition(),0), new MotionState(armTarget,0), maxVelocity,maxAcceleration);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity,maxAcceleration);
        newProfile = new TrapezoidProfile(constraints,new TrapezoidProfile.State(armMotor.getCurrentPosition(),0),new TrapezoidProfile.State(armTarget,0));

        waitForStart();

        previousTarget = armTarget;
        while(opModeIsActive()){
            motionComplete = Math.abs(armTarget - armMotor.getCurrentPosition()) < 50;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(armMotor.getCurrentPosition(),0), new MotionState(armTarget,0), maxVelocity,maxAcceleration);
            newProfile = new TrapezoidProfile(constraints,new TrapezoidProfile.State(armMotor.getCurrentPosition(),0),new TrapezoidProfile.State(armTarget,0));

            if(previousTarget != armTarget){
                previousTarget = armTarget;
                time.reset();
            }

            armCoefficients = new PIDCoefficientsEx(kp,ki,kd,maximumIntegralSum,stability,lowPassGain);
            armController = new PIDEx(armCoefficients);


            MotionState state = profile.get(time.time());
            TrapezoidProfile.State newState = newProfile.calculate(time.time());

            double instantTarget = state.getX();
            double instantVelocity = state.getV();
            //double instantAcceleration = state.getA();


            armPower = armController.calculate(instantTarget, armMotor.getCurrentPosition());


           armMotor.set(armPower);



            mainTelemetry.addData("Arm Pose: ", armMotor.getCurrentPosition());
            mainTelemetry.addData("Previous Target: ", previousTarget);
            mainTelemetry.addData("Arm Target: ", armTarget);
            mainTelemetry.addData("Instant Target: ", instantTarget);
            mainTelemetry.addData("Instant Velocity: ", instantVelocity);
            //mainTelemetry.addData("Instant Acceleration: ", instantAcceleration);
            mainTelemetry.addData("Max Jerk: ", maxJerk);
            mainTelemetry.addData("Arm Power: ", armPower);
            mainTelemetry.update();

        }
    }
}
