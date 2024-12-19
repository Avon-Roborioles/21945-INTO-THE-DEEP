package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

/**
 * This program is used to test out PID control of the arm
 */

@Config
@TeleOp(name="Arm Auto Test", group="Tests")
public class ArmAutoTest extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
    GamepadEx driverOp;

    //pid control
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double maximumIntegralSum = 1;
    public static double stabilityThreashold = 1;
    public static double lowPassGain = 1;


    public static int target = 0;
    public static int extendTarget = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad1);

        //init method
        arm.init(hardwareMap,driverOp,false);

        waitForStart();

        while(opModeIsActive()){
            //update method
            arm.setTarget(target);
            arm.update();

            arm.getTelemetryFULL(telemetry);
            telemetry.update();

        }

    }
}
