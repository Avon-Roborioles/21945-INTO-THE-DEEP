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


    double leftY;
    GamepadEx driverOp;
    ElapsedTime time;

    //returns the current target position based on max acceleration, max velocity, arm Target, & elapsed time
    public double motion_profile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {

        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time,2);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time,2);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad2);
        leftY = driverOp.getLeftY();
        time = new ElapsedTime();

        MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init PID
        Motor armMotor = new Motor(hardwareMap,"armMotor");
        armMotor.setInverted(true);
        armMotor.encoder.setDirection(Motor.Direction.REVERSE);
        armMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.stopAndResetEncoder();
        armMotor.setRunMode(Motor.RunMode.RawPower);
        PIDCoefficientsEx armCoefficients = new PIDCoefficientsEx(kp,ki,kd,maximumIntegralSum,stability,lowPassGain);
        PIDEx armController = new PIDEx(armCoefficients);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(armMotor.getCurrentPosition(),0), new MotionState(armTarget,0), maxVelocity,maxAcceleration);


        waitForStart();

        previousTarget = armTarget;



        while(opModeIsActive()){
            //loop
            motionComplete = Math.abs(armTarget - armMotor.getCurrentPosition()) < 50;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(armMotor.getCurrentPosition(),0), new MotionState(armTarget,0), maxVelocity,maxAcceleration);

            profile = new MotionProfile();

            if(previousTarget != armTarget){
                previousTarget = armTarget;
                time.reset();
            }
            armCoefficients = new PIDCoefficientsEx(kp,ki,kd,maximumIntegralSum,stability,lowPassGain);
            armController = new PIDEx(armCoefficients);


            MotionState state = profile.get(time.time());

            double instantTarget = state.getX();
            double instantVelocity = state.getV();
            double instantAcceleration = state.getA();

            armPower = armController.calculate(instantTarget,armMotor.getCurrentPosition());
           armMotor.set(armPower);

            //telemetry
            mainTelemetry.addData("Arm Pose: ", armMotor.getCurrentPosition());
            mainTelemetry.addData("Previous Target: ", previousTarget);
            mainTelemetry.addData("Arm Target: ", armTarget);
            mainTelemetry.addData("Instant Target: ", instantTarget);
            mainTelemetry.addData("Instant Velocity: ", instantVelocity);
            mainTelemetry.addData("Instant Acceleration: ", instantAcceleration);
            mainTelemetry.addData("Arm Power: ", armPower);
            mainTelemetry.addData("New Instant Target: ", instantTarget);
            mainTelemetry.update();
        }
    }
}
