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
    MotorEx slidesMotor;
    Servo intakeServo; //0 is open, 1 is close

    //TODO - absolute positions
    private final int fencePose = 0;
    private final int highRungPose = 0;
    private final int lowRungPose = 0;
    private final int maxPose = 0;

    private int currentSlidesPose;
    private int currentGrabberPose;
    private Slide_Modes slideMode;
    private boolean autoMode = false;
    private boolean pickup = false;

    //slides PID variables + intake target
    public double instantTarget = 0;
    public double slidesTarget = 0;
    public double slidesPower = 0;
    public double intakeTarget = 0;

    //Motion Profile + Full State Feedback PID Controller
    private final double kp = 0; //TODO position
    private final double ka = 0; //TODO velocity
    private final double MAX_VELOCITY = 0; //TODO
    private final double MAX_ACCELERATION = 0; //TODO
    MotionProfile motionProfile;
    Vector slidesCoefficients;
    FullStateFeedback slidesController;
    public ElapsedTime time;
    public boolean busy = false;

    //Control Variables
    GamepadEx driverOp;
    ToggleButtonReader y_button, a_button, x_button, b_button; //modes
    double rightY;

    //enum commands for slide modes
    public enum Slide_Modes {
        DRIVER_MODE,
        HOLD_MODE
    }

    //--------TELEOP COMMANDS---------
    public void init(HardwareMap hardwareMap, GamepadEx gamepad, boolean teleOp){
        driverOp = gamepad;
        time = new ElapsedTime();

        slidesCoefficients = new Vector(new double[] {kp,ka});
        slidesController = new FullStateFeedback(slidesCoefficients);

        //---initialize toggles & buttons---
        y_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.Y
        );
        a_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );
        x_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );
        b_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.B
        );

        //slides
        slidesMotor = new MotorEx(hardwareMap,"slidesMotor");
        //slidesMotor.setInverted(true);
        slidesMotor.encoder.setDirection(Motor.Direction.REVERSE);
        slidesMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slidesMotor.stopAndResetEncoder();
        slidesMotor.setRunMode(Motor.RunMode.RawPower);

        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(slidesMotor.getCurrentPosition(),0), new MotionState(slidesTarget,0), MAX_VELOCITY,MAX_ACCELERATION);

        //intake
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
    }

    private void updateToggles(){
        y_button.readValue();
        a_button.readValue();
        x_button.readValue();
        b_button.readValue();
    }

    public void run_teleOp(){} //TODO

    public void run_teleOp(Driver_Feedback feedback){} //TODO

    //--------AUTO COMMANDS------------
    public boolean IsBusy(){
        return instantTarget != slidesTarget;
    }

    public boolean intakeIsBusy(){
        return intakeServo.getPosition() != intakeTarget;
    }

    public void setTarget(int slides){
        slidesTarget = Math.min(slides, maxPose);
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(slidesMotor.getCurrentPosition(),0), new MotionState(slidesTarget,0), MAX_VELOCITY,MAX_ACCELERATION);
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
        //slides
        MotionState state = motionProfile.get(time.time());

        instantTarget = state.getX();
        double instantVelocity = state.getV();

        double measuredPosition = slidesMotor.getCurrentPosition();
        double measuredVelocity = slidesMotor.getVelocity(); //* -1;

        Vector measuredState = new Vector(new double[] {measuredPosition,measuredVelocity});
        Vector targetState = new Vector(new double[] {instantTarget,instantVelocity});

        try {
            slidesPower = slidesController.calculate(targetState,measuredState);
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        slidesMotor.setVelocity(-instantVelocity);

        //intake auto sequence
        if(autoMode){
            if(pickup){ //raise slides after intake is closed
                if(!intakeIsBusy()){
                    setTarget(highRungPose);
                    autoMode = false;
                }
            } else { //score - lower slides, wait .2 sec, then open intake
                if(!IsBusy()){
                    if(time.seconds() >= 0.2){
                        grab(false);
                        autoMode = false;
                    }
                }
            }
        }
    }

    public void getTelemetry(Telemetry telemetry){
        telemetry.addLine("----SLIDES DATA----");
        telemetry.addData("Lift Mode: ", slideMode);
        telemetry.addData("Lift Pose: ", slidesMotor.getCurrentPosition());
        telemetry.addData("Lift Velocity: ", slidesMotor.getVelocity());
        telemetry.addData("Lift Power: ", slidesPower);
        telemetry.addData("Lift Busy?: ", IsBusy());
        telemetry.addData("Specimen Intake Pose: ", intakeServo.getPosition());
        telemetry.addData("Specimen Intake Busy?: ", intakeIsBusy());
    }
}
