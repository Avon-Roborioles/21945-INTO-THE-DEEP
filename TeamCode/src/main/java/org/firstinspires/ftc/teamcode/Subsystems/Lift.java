package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FullStateFeedback;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//robot subsystem for slides + specimen intake
public class Lift {
    //motor objects & related variables
    MotorEx liftMotor;
    Servo intakeServo; //0 is open, 1 is close

    //TODO - absolute positions
    private final int fencePose = 0;
    private final int highRungPose = 1700;
    private final int lowRungPose = 1400;
    private int maxPose = 2200;
    private int groundPose = 0;

    private int currentLiftPose;
    private int currentGrabberPose;
    private Lift_Modes liftMode = Lift_Modes.HOLD_MODE;
    private boolean autoMode = false;
    private boolean pickup = false;

    //slides PID variables + intake target
    public double instantTarget = 0;
    public double liftTarget = 0;
    public double liftPower = 0;
    public double intakeTarget = 0;

    //Motion Profile + Full State Feedback PID Controller
    private final double kp = 0.009;
    private final double ka = 0.0001;
    private final double MAX_VELOCITY = 2000;
    private final double MAX_ACCELERATION = 2000;
    private final double maxPower = 0.8;
    MotionProfile motionProfile;
    Vector liftCoefficients;
    FullStateFeedback liftController;
    public ElapsedTime time;
    public boolean busy = false;

    //Control Variables
    GamepadEx driverOp;
    ToggleButtonReader leftBumper, rightBumper, a_button, x_button, b_button;
    double rightY;

    //enum commands for slide modes
    public enum Lift_Modes {
        DRIVER_MODE,
        HOLD_MODE,
        AUTO_MODE
    }

    //--------TELEOP COMMANDS---------
    public void init(HardwareMap hardwareMap, GamepadEx gamepad, boolean teleOp){
        driverOp = gamepad;
        time = new ElapsedTime();
        liftCoefficients = new Vector(new double[] {kp,ka});
        liftController = new FullStateFeedback(liftCoefficients);

       leftBumper = new ToggleButtonReader(driverOp, GamepadKeys.Button.LEFT_BUMPER);
       rightBumper = new ToggleButtonReader(driverOp, GamepadKeys.Button.RIGHT_BUMPER);
       a_button = new ToggleButtonReader(driverOp, GamepadKeys.Button.A);
       x_button = new ToggleButtonReader(driverOp, GamepadKeys.Button.X);
       b_button = new ToggleButtonReader(driverOp, GamepadKeys.Button.B);

        //slides
        liftMotor = new MotorEx(hardwareMap,"liftMotor");
        liftMotor.setRunMode(Motor.RunMode.RawPower);
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        liftMotor.setInverted(true);
//        liftMotor.encoder.setDirection(Motor.Direction.REVERSE);

        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(liftMotor.getCurrentPosition(),0), new MotionState(liftTarget,0), MAX_VELOCITY,MAX_ACCELERATION);
    }

    private void updateToggles(){
       leftBumper.readValue();
       rightBumper.readValue();
       a_button.readValue();
       x_button.readValue();;
       b_button.readValue();
    }

    public void run_teleOp(){
        //control logic
        currentLiftPose = liftMotor.getCurrentPosition();

        //control
        if(leftBumper.isDown()){
            if(currentLiftPose < maxPose){
                liftMode = Lift_Modes.DRIVER_MODE;
                setTarget(currentLiftPose);
                liftPower = 0.9;
            } else {
                liftMode = Lift_Modes.HOLD_MODE;
            }
        } else if(rightBumper.isDown()){
            if(currentLiftPose > groundPose){
                liftMode = Lift_Modes.DRIVER_MODE;
                setTarget(currentLiftPose);
                liftPower = -0.3;
            } else liftMode = Lift_Modes.HOLD_MODE;
        } else {
            liftMode = Lift_Modes.HOLD_MODE;
        }

        //preset
        if(x_button.wasJustPressed()){
            setTarget(lowRungPose);
            liftMode = Lift_Modes.HOLD_MODE;
        }
        if(b_button.wasJustPressed()){
            setTarget(highRungPose);
            liftMode = Lift_Modes.HOLD_MODE;
        }
        if(a_button.wasJustPressed()){
            setTarget(fencePose);
            liftMode = Lift_Modes.HOLD_MODE;
        }

        //modes
        if(liftMode == Lift_Modes.HOLD_MODE){
            MotionState state = motionProfile.get(time.time());

            double instantTarget = state.getX();
            double instantVelocity = state.getV();

            double measuredVelocity = liftMotor.getVelocity() * -1;

            Vector measuredState = new Vector(new double[] {currentLiftPose,measuredVelocity});
            Vector targetState = new Vector(new double[] {instantTarget,instantVelocity});

            try {
                liftPower = liftController.calculate(targetState,measuredState);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        liftMotor.set(liftPower);

        //haptic feedback

        updateToggles();
    }

    public void run_teleOp(Driver_Feedback feedback){
       //control logic
        currentLiftPose = liftMotor.getCurrentPosition();
        double velocityAddon = liftMotor.getVelocity() * 0.05;

        if(leftBumper.isDown()){
            setTarget(currentLiftPose);
           liftMode = Lift_Modes.DRIVER_MODE;
            liftPower = -0.4;

        } else if(rightBumper.isDown()){
            if(currentLiftPose < maxPose) {
                setTarget(currentLiftPose);
                liftMode = Lift_Modes.DRIVER_MODE;
                liftPower = 0.6;
            }
        } else {
            liftPower = 0.04;
        }

        //preset
        if(x_button.wasJustPressed()){
            setTarget(lowRungPose);
            liftMode = Lift_Modes.HOLD_MODE;
            feedback.alert_side(false, driverOp);
        }
        if(b_button.wasJustPressed()){
            setTarget(highRungPose);
            liftMode = Lift_Modes.HOLD_MODE;
            feedback.alert_side(false, driverOp);
        }
        if(a_button.wasJustPressed()){
            setTarget(fencePose);
            liftMode = Lift_Modes.HOLD_MODE;
            feedback.alert_side(false, driverOp);
        }
//
//        //modes
        if(liftMode == Lift_Modes.HOLD_MODE){
            update();

        } else {
            liftMotor.set(liftPower);

        }


       //haptic feedback
        updateToggles();
    }

    //--------AUTO COMMANDS------------
    public boolean IsBusy(){
        return Math.abs(liftTarget - instantTarget) > 50;
    }

    public boolean intakeIsBusy(){
        return intakeServo.getPosition() != intakeTarget;
    }

    public void setTarget(double target){
        liftTarget = target;
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(liftMotor.getCurrentPosition(),0), new MotionState(liftTarget,0), MAX_VELOCITY,MAX_ACCELERATION);
        time.reset();
    }

    public void grab(boolean close){
        if(close){
            intakeServo.setPosition(1);
        } else {
            intakeServo.setPosition(0);
        }
    }

    public void pickup(){ //close intake, then raise slides up
        time.reset();
        grab(true);
        autoMode = true;
        pickup = true;
    }

    public void score(boolean highRung){ //lower slides down, then open intake
        time.reset();
        if(highRung) {
            setTarget(highRungPose);
        } else {
            setTarget(lowRungPose);
        }
        autoMode = true;
        pickup = false;
    }

    public void update(){
        MotionState state = motionProfile.get(time.time());

        instantTarget = state.getX();
        double instantVelocity = state.getV();

        double measuredPosition = liftMotor.getCurrentPosition();
        double measuredVelocity = liftMotor.getVelocity() * -1;

        Vector measuredState = new Vector(new double[] {measuredPosition,measuredVelocity});
        Vector targetState = new Vector(new double[] {instantTarget,instantVelocity});

        try {
            liftPower = liftController.calculate(targetState,measuredState);
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        //liftMotor.setVelocity(-instantVelocity);
        liftMotor.set(liftPower);

//        //intake auto sequence
//        if(autoMode){
//            if(pickup){ //raise slides after intake is closed
//                if(!intakeIsBusy()){
//                    setTarget(highRungPose);
//                    autoMode = false;
//                }
//            } else { //score - lower slides, wait .2 sec, then open intake
//                if(!IsBusy()){
//                    if(time.seconds() >= 0.2){
//                        grab(false);
//                        autoMode = false;
//                    }
//                }
//            }
//        }
    }

    public void getTelemetry(Telemetry telemetry){
        telemetry.addLine("----LIFT DATA----");
        telemetry.addData("Lift Mode: ", liftMode);
        telemetry.addData("Lift Pose: ", liftMotor.getCurrentPosition());
        telemetry.addData("Lift Instant Target: ", instantTarget);
        telemetry.addData("Lif Target: ", liftTarget);
        telemetry.addData("Lift Velocity: ", liftMotor.getVelocity());
        telemetry.addData("Lift Power: ", liftPower);
        telemetry.addData("Lift Busy?: ", IsBusy());
//        telemetry.addData("Specimen Intake Pose: ", intakeServo.getPosition());
//        telemetry.addData("Specimen Intake Busy?: ", intakeIsBusy());
    }
}
