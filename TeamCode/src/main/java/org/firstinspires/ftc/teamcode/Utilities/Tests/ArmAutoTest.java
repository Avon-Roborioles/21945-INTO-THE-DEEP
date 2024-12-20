package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@Config
@TeleOp(name="Arm Auto Test", group="Tests")
public class ArmAutoTest extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();
    GamepadEx driverOp;

    //pid control
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    double maximumIntegralSum = 0;
    double stabilityThreashold = 0;
    double lowPassGain = 0;


    public static int armTarget = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad1);

        arm.init(hardwareMap, driverOp,false);

        while(opModeInInit()){
            arm.getTelemetryBRIEF(telemetry);
        }

        waitForStart();

        while(opModeIsActive()){
            //arm.goTo(armTarget);
            arm.setTarget(armTarget);
            arm.update();
            arm.getTelemetryFULL(telemetry);
            telemetry.update();

        }

    }
}
