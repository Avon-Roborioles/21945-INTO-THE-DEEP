//helps Android Studio know where file is located within our Repository
package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.tuning.FollowerConstants;

public class Drivetrain {
    //motor objects
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    //FTC Lib & Pedro-Pathing objects
    Follower pedroDrivetrain;
    boolean teleOpDrive = true;

    IMU imu;
    GamepadEx driverOp;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection;
    //Telemetry Data
    double strafeSpeed = 0;
    double forwardSpeed = 0;
    double turnSpeed = 0;
    double gyroAngle = 0;
    int strength = 0;
    boolean robotCentricMode = true;
    double speedLimit = 1;


    ToggleButtonReader y_button, a_button; //drive modes
    ToggleButtonReader d_up, d_down, d_left, d_right; //speed control

    //initializes the drivetrain
    public void init(HardwareMap hardwareMap, GamepadEx gamepad){
        pedroDrivetrain = new Follower(hardwareMap);

        //imu = hardwareMap.get(IMU.class, "imu");
        driverOp = gamepad;

        y_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.Y
        );
        a_button = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );
        d_up = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_UP
        );
        d_down = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_DOWN
        );
        d_left = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_LEFT
        );
        d_right = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_RIGHT
        );


        //adding snappy breaking to drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pedroDrivetrain.startTeleopDrive();
    }

    private void updateToggles(){
        y_button.readValue();
        a_button.readValue();
        d_up.readValue();
        d_right.readValue();
        d_left.readValue();
        d_down.readValue();
    }

    public void run_teleOp(Driver_Feedback feedback){
        if(teleOpDrive) {
            strafeSpeed = -driverOp.getLeftX() * speedLimit; //changed to negative to fix inverted controls
            forwardSpeed = driverOp.getLeftY() * speedLimit;
            turnSpeed = -driverOp.getRightX() * speedLimit;
            //gyroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double strafeAbsolute = Math.abs(strafeSpeed);
            double forwardAbsolute = Math.abs(forwardSpeed);
            double turnAbsolute = Math.abs(turnSpeed);


            //Robot Centric and Field Centric Modes
            if (y_button.wasJustPressed()) { //robot
                robotCentricMode = true;

            } else if (a_button.wasJustPressed()) { //field
                robotCentricMode = false;
            }

            //Speed Controls - set top speed percentage
            if (d_up.wasJustPressed()) {
                speedLimit = 1;
            } else if (d_right.wasJustPressed()) {
                speedLimit = 0.7;
            } else if (d_left.wasJustPressed()) {
                speedLimit = 0.5;
            } else if (d_down.wasJustPressed()) {
                speedLimit = 0.3;
            }


            //Pedro-Pathing TeleOp Control
            pedroDrivetrain.setTeleOpMovementVectors(forwardSpeed, strafeSpeed, turnSpeed, robotCentricMode);
        } else {
            //logic to get out of autoDrive
        }

        pedroDrivetrain.update();
        updateToggles();


        //TODO Driver Alerts for Speed
//        if(strafeAbsolute == 1 || forwardAbsolute == 1 || turnAbsolute == 1){
//            strength = 100;
//            feedback.alert_driver(driverOp, strength);
//        } else if (strafeAbsolute > 0.9 || forwardAbsolute > 0.9 || turnAbsolute > 0.9) {
//            strength = 70;
//            feedback.alert_driver(driverOp, strength);
//        } else if (strafeAbsolute > 0.8 || forwardAbsolute > 0.8 || turnAbsolute > 0.8) {
//            strength = 40;
//            feedback.alert_driver(driverOp, strength);
//        } else {
//            strength = 0;
//        }
    }


    //most important info of drivetrain to reduce clutter
    public void getTelemetryBRIEF(Telemetry telemetry){
        telemetry.addLine("---Drivetrain Control Data---");
        telemetry.addData("Strafe Speed: ", strafeSpeed);
        telemetry.addData("Forward Speed: ", turnSpeed);
        telemetry.addData("Turn Speed: ", turnSpeed);
        //telemetry.addData("Gyro Angle (IMU): ", gyroAngle);
    }

    //all data metrics from drivetrain for testing purposes
    public void getTelemetryFULL(Telemetry telemetry){
        telemetry.addLine("---Drivetrain Motor Powers---");
//        telemetry.addData("FL Power: ", leftFront.get());
//        telemetry.addData("FR Power: ", rightFront.get());
//        telemetry.addData("RL Power: ", leftRear.get());
//        telemetry.addData("RR Power: ", rightRear.get());

        telemetry.addLine("---Drivetrain Control Data---");
        telemetry.addData("Strafe Speed: ", strafeSpeed);
        telemetry.addData("Forward Speed: ", turnSpeed);
        telemetry.addData("Turn Speed: ", turnSpeed);
        //telemetry.addData("Gyro Angle (IMU): ", gyroAngle);

        telemetry.addLine("---Drivetrain Feedback Data---");
        telemetry.addData("Feedback Strength: ", strength);
    }
}