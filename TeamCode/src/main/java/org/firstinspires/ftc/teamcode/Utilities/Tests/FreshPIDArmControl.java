package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Line;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="Fresh Arm PID Control", group="Tests")
public class FreshPIDArmControl extends LinearOpMode {

    public static double armTarget = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double maximumIntegralSum = 0;
    public static double stability = 0;
    public static double lowPassGain = 0;

    double leftY;
    GamepadEx driverOp;

    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad2);
        leftY = driverOp.getLeftY();

        Motor armMotor = new Motor(hardwareMap,"armMotor");
        MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor.setInverted(true);
        armMotor.encoder.setDirection(Motor.Direction.REVERSE);
        armMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.stopAndResetEncoder();
        armMotor.setRunMode(Motor.RunMode.RawPower);

        PIDCoefficientsEx armCoefficients = new PIDCoefficientsEx(kp,ki,kd,maximumIntegralSum,stability,lowPassGain);
        PIDEx armController = new PIDEx(armCoefficients);

        waitForStart();


        while(opModeIsActive()){
            armCoefficients = new PIDCoefficientsEx(kp,ki,kd,maximumIntegralSum,stability,lowPassGain);
            armController = new PIDEx(armCoefficients);

            double armPower = armController.calculate(armTarget,armMotor.getCurrentPosition());
            armMotor.set(armPower);

            mainTelemetry.addData("Arm Pose: ", armMotor.getCurrentPosition());
            mainTelemetry.addData("Arm Target: ", armTarget);
            mainTelemetry.addData("Arm Power: ", armPower);
            mainTelemetry.update();

        }
    }
}
