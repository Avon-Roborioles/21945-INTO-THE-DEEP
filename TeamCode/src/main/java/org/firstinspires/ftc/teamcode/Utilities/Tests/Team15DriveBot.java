package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Team15DriveBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //init variables
        MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx driverOp = new GamepadEx(gamepad1);
        Boolean robotCentricMode = true;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        ToggleButtonReader y_button = new ToggleButtonReader(driverOp, GamepadKeys.Button.Y); //robot centric
        ToggleButtonReader a_button = new ToggleButtonReader(driverOp, GamepadKeys.Button.A); //field centric
        ToggleButtonReader b_button = new ToggleButtonReader(driverOp, GamepadKeys.Button.B); //field centric


        Motor leftFront = new Motor(hardwareMap,"leftFront");
        Motor leftRear = new Motor(hardwareMap,"leftRear");
        Motor rightFront = new Motor(hardwareMap,"rightFront");
        Motor rightRear = new Motor(hardwareMap,"rightRear");


        MecanumDrive drivetrain = new MecanumDrive(true,leftFront,rightFront,leftRear,rightRear);

        waitForStart();

        while(opModeIsActive()){
           double strafeSpeed = -driverOp.getLeftX();
           double forwardSpeed = driverOp.getLeftY();
           double turnSpeed = -driverOp.getRightX();
           double gyroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

           if(y_button.wasJustPressed()){
               robotCentricMode = true;
           } else if(a_button.wasJustPressed()){
               robotCentricMode = false;
           }

           if(b_button.wasJustPressed()){
               imu.resetYaw();
           }

           if(robotCentricMode) {
               drivetrain.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, true);
           } else {
               drivetrain.driveFieldCentric(strafeSpeed,forwardSpeed,turnSpeed,gyroAngle, true);
           }

           //bot controls with telemetry
           mainTelemetry.addLine("----Roborioles Drive Bot----");
           mainTelemetry.addLine();
           mainTelemetry.addLine("Control the Robot with the Left and Right Joysticks");
           mainTelemetry.addLine("Press the Y button to enable Robot Centric Driving");
           mainTelemetry.addLine("Press the A Button to enable Field Centric Driving");
           mainTelemetry.addLine("Press B to reset Field Centric Driving (Point Robot away from you when resetting");
           mainTelemetry.addLine();
           mainTelemetry.addData("Robot Centric Mode?: ", robotCentricMode);
           mainTelemetry.addData("Strafe Speed: ", strafeSpeed);
           mainTelemetry.addData("Forward Speed: ", forwardSpeed);
           mainTelemetry.addData("Turn Speed: ", turnSpeed);
           mainTelemetry.addData("Gyro Angle: ", gyroAngle);
           mainTelemetry.addData("Drivetrain Info: ", drivetrain.toString());
           mainTelemetry.update();

           y_button.readValue();
           a_button.readValue();
           b_button.readValue();
        }
    }
}
