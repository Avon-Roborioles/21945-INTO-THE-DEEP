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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Fresh Arm PID Control", group="Tests")
public class FreshPIDArmControl extends LinearOpMode {

    public double previousTarget = 0;
    public static double armTarget = 0;
    public static double extendTarget = 0;
    public double armPower = 0;
    public double extendPower = 0;
    public static double kpArm = 0.002; //helps match arm position with instant position
    public static double kpExtend = 0.01;
    public static double kiExtend = 0;
    public static double kdExtend = 0;
    public static double ka = 0.0004; //helps match acceleration with instant acceleration
    public static double maxVelocity = 2000;
    public static double maxAcceleration = 4000;
    public static double maxJerk = 0;
    public boolean motionComplete = true;
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
        MotorEx armMotor = new MotorEx(hardwareMap,"armMotor");
        armMotor.setInverted(true);
        armMotor.encoder.setDirection(Motor.Direction.REVERSE);
        armMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.stopAndResetEncoder();
        armMotor.setRunMode(Motor.RunMode.RawPower);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(armMotor.getCurrentPosition(),0), new MotionState(armTarget,0), maxVelocity,maxAcceleration);

        MotorEx extendMotor = new MotorEx(hardwareMap,"extensionMotor");
        extendMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendMotor.stopAndResetEncoder();
        extendMotor.setRunMode(Motor.RunMode.RawPower);

        waitForStart();

        previousTarget = armTarget;

        while(opModeIsActive()){
            //loop
            motionComplete = Math.abs(armTarget - armMotor.getCurrentPosition()) < 50;
            boolean overshoot = false;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(armMotor.getCurrentPosition(),0), new MotionState(armTarget,0), maxVelocity,maxAcceleration,maxJerk,overshoot);


            if(previousTarget != armTarget){
                previousTarget = armTarget;
                time.reset();
            }

            //---------------------------------
            Vector armCoefficients = new Vector(new double[] {kpArm, ka});
            FullStateFeedback armController = new FullStateFeedback(armCoefficients);

            //extend controller
            PIDCoefficients extendCoefficients = new PIDCoefficients(kpExtend,kiExtend,kdExtend);
            BasicPID extendController = new BasicPID(extendCoefficients);

            MotionState state = profile.get(time.time());

            double instantTarget = state.getX();
            double instantVelocity = state.getV();
            double instantAcceleration = state.getA();

            double measuredPosition = armMotor.getCurrentPosition();
            double measuredVelocity = armMotor.getVelocity() * -1;
            double measuredAcceleration = armMotor.getAcceleration() * -1;

            Vector measuredState = new Vector(new double[] {measuredPosition,measuredVelocity});
            Vector targetState = new Vector(new double[] {instantTarget,instantVelocity});

            try {
                armPower = armController.calculate(targetState,measuredState);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            double measuredExtendPosition = extendMotor.getCurrentPosition();
            extendPower = extendController.calculate(extendTarget,measuredExtendPosition);

            armMotor.setVelocity(-instantVelocity);
            armMotor.set(armPower);

            extendMotor.set(extendPower);

            //telemetry
            mainTelemetry.addData("Arm Pose: ", armMotor.getCurrentPosition());
            mainTelemetry.addData("Previous Target: ", previousTarget);
            mainTelemetry.addData("Instant Target: ", instantTarget);
            mainTelemetry.addData("Arm Target: ", armTarget);
            mainTelemetry.addData("Extend Target: ", extendTarget);
            mainTelemetry.addData("Extend Pose: ", measuredExtendPosition);
            mainTelemetry.addData("Instant Velocity: ", instantVelocity);
            mainTelemetry.addData("Measured Velocity: ", measuredVelocity);
            mainTelemetry.addData("Instant Acceleration: ", instantAcceleration);
            mainTelemetry.addData("Measured Acceleration: ", measuredAcceleration);
            mainTelemetry.addData("Arm Power: ", armPower);
            mainTelemetry.update();
        }
    }
}
