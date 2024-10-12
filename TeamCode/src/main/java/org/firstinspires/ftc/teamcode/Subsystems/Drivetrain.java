//helps Android Studio know where file is located within our Repository
package org.firstinspires.ftc.teamcode.Subsystems;

//import needed libraries
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.*;

public class Drivetrain {
    //motor objects
    Motor leftFront = null;
    Motor rightFront = null;
    Motor leftRear = null;
    Motor rightRear = null;
    //FTC Lib objects
    MecanumDrive drivetrain;
    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection;
    //Telemetry Data
    double strafeSpeed = 0;
    double forwardSpeed = 0;
    double turnSpeed = 0;
    double gyroAngle = 0;
    int strength = 0;

    //initializes the drivetrain
    public void init(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");

        leftFront = new Motor(hardwareMap,"leftFront");
        rightFront = new Motor(hardwareMap,"rightFront");
        leftRear = new Motor(hardwareMap,"leftRear");
        rightRear = new Motor(hardwareMap,"rightRear");

        //helps orient the robot for the IMU, change whenever the control hub is rotated
        logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        drivetrain = new MecanumDrive(leftFront, rightFront,leftRear, rightRear);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void run_fieldCentric(GamepadEx driverOp, Driver_Feedback feedback){
        //update Telemetry Variables
        strafeSpeed = driverOp.getLeftX() ; //changed to negative to fix inverted controls
        forwardSpeed = driverOp.getLeftY();
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


        //driver feedback functions
        if(strafeAbsolute == 1 || forwardAbsolute == 1 || turnAbsolute == 1){
            strength = 100;
            feedback.alert_driver(driverOp, strength);
        } else if (strafeAbsolute > 0.9 || forwardAbsolute > 0.9 || turnAbsolute > 0.9) {
            strength = 70;
            feedback.alert_driver(driverOp, strength);
        } else if (strafeAbsolute > 0.8 || forwardAbsolute > 0.8 || turnAbsolute > 0.8) {
            strength = 40;
            feedback.alert_driver(driverOp, strength);
        } else {
            strength = 0;
        }

    }

    //drivetrain teleOp with no feedback
    public void run_fieldCentric(GamepadEx driverOp){
        //update Telemetry Variables
        strafeSpeed = driverOp.getLeftX() ; //changed to negative to fix inverted controls
        forwardSpeed = driverOp.getLeftY();
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
        telemetry.addData("Gyro Angle (IMU): ", gyroAngle);
    }

    //all data metrics from drivetrain for testing purposes
    public void getTelemetryFULL(Telemetry telemetry){
        telemetry.addLine("---Drivetrain Motor Powers---");
        telemetry.addData("FL Power: ", leftFront.get());
        telemetry.addData("FR Power: ", rightFront.get());
        telemetry.addData("RL Power: ", leftRear.get());
        telemetry.addData("RR Power: ", rightRear.get());

        telemetry.addLine("---Drivetrain Control Data---");
        telemetry.addData("Strafe Speed: ", strafeSpeed);
        telemetry.addData("Forward Speed: ", turnSpeed);
        telemetry.addData("Turn Speed: ", turnSpeed);
        telemetry.addData("Gyro Angle (IMU): ", gyroAngle);

        telemetry.addLine("---Drivetrain Feedback Data---");
        telemetry.addData("Feedback Strength: ", strength);
    }
}