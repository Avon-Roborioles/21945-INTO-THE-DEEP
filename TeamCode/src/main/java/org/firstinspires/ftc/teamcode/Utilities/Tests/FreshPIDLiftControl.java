package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FullStateFeedback;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
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
    public double liftPower = 0;
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
        liftMotor.setRunMode(Motor.RunMode.RawPower);
        liftMotor.setInverted(true);
        liftMotor.encoder.setDirection(Motor.Direction.REVERSE);

        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(liftMotor.getCurrentPosition(),0), new MotionState(liftTarget,0), maxVelocity,maxAcceleration);

        waitForStart();

        previousTarget = liftTarget;

        while(opModeIsActive()){
            motionComplete = Math.abs(liftTarget - liftMotor.getCurrentPosition()) < 50;
            boolean overshoot = false;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(liftMotor.getCurrentPosition(),0), new MotionState(liftTarget,0), maxVelocity,maxAcceleration,maxJerk,overshoot);


            if(previousTarget != liftTarget){
                previousTarget = liftTarget;
                time.reset();
            }

            //---------------------------------
            Vector liftCoefficients = new Vector(new double[] {kp, ka});
            FullStateFeedback armController = new FullStateFeedback(liftCoefficients);

            MotionState state = profile.get(time.time());

            double instantTarget = state.getX();
            double instantVelocity = state.getV();
            double instantAcceleration = state.getA();

            double measuredPosition = liftMotor.getCurrentPosition();
            double measuredVelocity = liftMotor.getVelocity() * -1;
            double measuredAcceleration = liftMotor.getAcceleration() * -1;

            Vector measuredState = new Vector(new double[] {measuredPosition,measuredVelocity});
            Vector targetState = new Vector(new double[] {instantTarget,instantVelocity});

            try {
                liftPower = armController.calculate(targetState,measuredState);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            liftMotor.setVelocity(-instantVelocity);//(-instantVelocity);
            liftMotor.set(liftPower);


            //telemetry
            mainTelemetry.addData("Lift Pose: ", liftMotor.getCurrentPosition());
            mainTelemetry.addData("Previous Target: ", previousTarget);
            mainTelemetry.addData("Instant Target: ", instantTarget);
            mainTelemetry.addData("Lift Target: ", liftTarget);
            mainTelemetry.addData("Instant Velocity: ", instantVelocity);
            mainTelemetry.addData("Measured Velocity: ", measuredVelocity);
            mainTelemetry.addData("Instant Acceleration: ", instantAcceleration);
            mainTelemetry.addData("Measured Acceleration: ", measuredAcceleration);
            mainTelemetry.addData("Lift Power: ", liftPower);
            mainTelemetry.update();
        }

    }
}
