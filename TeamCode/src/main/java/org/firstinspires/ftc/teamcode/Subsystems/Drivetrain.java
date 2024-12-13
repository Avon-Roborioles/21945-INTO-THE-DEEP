//helps Android Studio know where file is located within our Repository
package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.follower.Follower;

public class Drivetrain {
    //motor objects
    Motor leftFront = null;
    Motor rightFront = null;
    Motor leftRear = null;
    Motor rightRear = null;

    //FTC Lib & Pedro-Pathing objects
    MecanumDrive drivetrain;
    Follower pedroDrivetrain;

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

//        leftFront = new Motor(hardwareMap,"leftFront");
//        rightFront = new Motor(hardwareMap,"rightFront");
//        leftRear = new Motor(hardwareMap,"leftRear");
//        rightRear = new Motor(hardwareMap,"rightRear");
//
//        //helps orient the robot for the IMU, change whenever the control hub is rotated
//        logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT; //RIGHT
//        drivetrain = new MecanumDrive(leftFront, rightFront,leftRear, rightRear);
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));

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


        pedroDrivetrain.startTeleopDrive(); //TODO
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
        strafeSpeed = -driverOp.getLeftX() * speedLimit; //changed to negative to fix inverted controls
        forwardSpeed = driverOp.getLeftY() * speedLimit;
        turnSpeed = -driverOp.getRightX() * speedLimit;
        //gyroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double strafeAbsolute = Math.abs(strafeSpeed);
        double forwardAbsolute = Math.abs(forwardSpeed);
        double turnAbsolute = Math.abs(turnSpeed);


        //Robot Centric and Field Centric Modes
        if(y_button.wasJustPressed()){ //robot
            robotCentricMode = true;

        } else if(a_button.wasJustPressed()){ //field
            robotCentricMode = false;
        }

        //Speed Controls - set top speed percentage
        if(d_up.wasJustPressed()){
            speedLimit = 1;
        } else if(d_right.wasJustPressed()){
            speedLimit = 0.7;
        } else if(d_left.wasJustPressed()){
            speedLimit = 0.5;
        } else if(d_down.wasJustPressed()){
            speedLimit = 0.1;
        }


        //Pedro-Pathing TeleOp Control
        pedroDrivetrain.setTeleOpMovementVectors(forwardSpeed,strafeSpeed,turnSpeed, robotCentricMode);
        pedroDrivetrain.update();
        updateToggles();
//        //uses FTCLib Library to control all logic of Field Centric Driving
//        if(robotCentricMode){
//           drivetrain.driveRobotCentric(
//                   strafeSpeed,
//                forwardSpeed,
//                turnSpeed);
//
//        } else {
//            drivetrain.driveFieldCentric(
//                    strafeSpeed,
//                    forwardSpeed,
//                    turnSpeed,
//                    gyroAngle
//            );
//        }

        //absolute values of driver inputs used for haptic feedback functions



        //driver feedback functions

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

    public void run_fieldCentric(){
        //update Telemetry Variables
        strafeSpeed = driverOp.getLeftX() * -1; //changed to negative to fix inverted controls
        forwardSpeed = driverOp.getLeftY() * -1;
        turnSpeed = driverOp.getRightX() * -1;
        gyroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        //uses FTCLib Library to control all logic of Field Centric Driving
        drivetrain.driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                turnSpeed,
                gyroAngle
        );

        //absolute values of driver inputs used for haptic feedback functions
        double strafeAbsolute = Math.abs(strafeSpeed);
        double forwardAbsolute = Math.abs(forwardSpeed);
        double turnAbsolute = Math.abs(turnSpeed);

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