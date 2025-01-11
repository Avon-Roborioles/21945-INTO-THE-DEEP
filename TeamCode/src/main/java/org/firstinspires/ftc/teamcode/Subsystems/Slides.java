package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FullStateFeedback;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//robot subsystem for slides + specimen intake
public class Slides {
    //motor objects & related variables
    MotorEx slidesMotor;
    Servo grabber;

    //TODO - absolute positions
    private final int fencePose = 0;
    private final int highRungPose = 0;
    private final int lowRungPose = 0;
    private final int maxPose = 0;

    private int currentSlidesPose;
    private int currentGrabberPose;

    //slides PID variables
    public double instantTarget = 0;
    public double slidesTarget = 0;
    public double slidesPower = 0;

    //Motion Profile + Full State Feedback PID Controller
    private final double kp = 0; //position
    private final double ka = 0; //velocity
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
    ToggleButtonReader d_up, d_down, d_left, d_right; //height toggles
    double rightY;

    //--------TODO TELEOP COMMANDS---------
    public void init(HardwareMap hardwareMap, GamepadEx gamepad, boolean teleOp){

    }

    private void updateToggles(){
    }

    public void run_teleOp(){}

    public void run_teleOp(Driver_Feedback feedback){}

    //--------TODO AUTO COMMANDS------------
    public boolean SlidesIsBusy(){
        return instantTarget != slidesTarget;
    }

    public void setTarget(int slides){
    }

    public void grab(){}

    public void pickup(){};

    public void score(){}

    public void update(){}

    public void getTelemetry(Telemetry telemetry){}
}
