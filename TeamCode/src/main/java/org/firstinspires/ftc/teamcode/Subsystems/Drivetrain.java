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

    //initializes the drivetrain
    public void init(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");

        //"DeviceName" MUST match name of motors in driver hub configuration
        //leftFront = hardwareMap.get(Motor.class, "leftFront");
        leftFront = new Motor(hardwareMap,"leftFront");
        //rightFront = hardwareMap.get(Motor.class, "rightFront");
        rightFront = new Motor(hardwareMap,"rightFront");
        //leftRear = hardwareMap.get(Motor.class, "leftRear");
        leftRear = new Motor(hardwareMap,"leftRear");
        //rightRear = hardwareMap.get(Motor.class, "rightRear");
        rightRear = new Motor(hardwareMap,"rightRear");

        //helps orient the robot for the IMU, change whenever the control hub is rotated
        logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        drivetrain = new MecanumDrive(leftFront, rightFront,leftRear, rightRear);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

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
    }
}

