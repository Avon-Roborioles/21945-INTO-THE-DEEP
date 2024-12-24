package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FullStateFeedback;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
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
    public static double kp = 0.03;
    public static double ki = 0;
    public static double kd = 0;
    public static double ks = 0.00002;
    public static double kV = 130; //used to help match motion profile velocity to usable value
    public static double maxVelocity = 20;
    public static double maxAcceleration = 20;
    public static double maximumIntegralSum = 0;
    public static double stability = 0;
    public static double lowPassGain = 0;
    public static double maxJerk = 0;
    public boolean motionComplete = true;
    double startTime = System.currentTimeMillis();
    private MotionProfile profile;



    double leftY;
    GamepadEx driverOp;
    ElapsedTime time;

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
            if(previousTarget != armTarget){
                previousTarget = armTarget;
                time.reset();
            }

            //---------------------------------
            armCoefficients = new PIDCoefficientsEx(kp,ki,kd,maximumIntegralSum,stability,lowPassGain);
            armController = new PIDEx(armCoefficients);

            Vector coefficients = new Vector(new double[] {kp,ks});
            FullStateFeedback newArmController = new FullStateFeedback(coefficients);

            MotionState state = profile.get(time.time());

            double instantTarget = state.getX();
            double instantVelocity = state.getV() * kV;
            double instantAcceleration = state.getA();

            double measuredPosition = armMotor.getCurrentPosition();
            double measuredVelocity = armMotor.getCorrectedVelocity() * -1;

            Vector measuredState = new Vector(new double[] {measuredPosition,measuredVelocity});
            Vector targetState = new Vector(new double[] {instantTarget,instantVelocity});

            //armPower = armController.calculate(instantTarget, armMotor.getCurrentPosition());
            try {
                armPower = newArmController.calculate(targetState,measuredState);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
            armMotor.set(armPower);

            //telemetry
            mainTelemetry.addData("Arm Pose: ", armMotor.getCurrentPosition());
            mainTelemetry.addData("Previous Target: ", previousTarget);
            mainTelemetry.addData("Arm Target: ", armTarget);
            mainTelemetry.addData("Instant Target: ", instantTarget);
            mainTelemetry.addData("Instant Velocity: ", instantVelocity);
            mainTelemetry.addData("Measured Velocity: ", measuredVelocity);
            mainTelemetry.addData("Instant Acceleration: ", instantAcceleration);
            mainTelemetry.addData("Arm Power: ", armPower);
            mainTelemetry.update();
        }
    }
}
