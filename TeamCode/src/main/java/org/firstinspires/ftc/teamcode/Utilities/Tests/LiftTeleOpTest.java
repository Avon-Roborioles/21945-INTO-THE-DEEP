package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@TeleOp
public class LiftTeleOpTest extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx driverOp = new GamepadEx(gamepad2);
        arm.init(hardwareMap,driverOp,true);

        MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        while(opModeIsActive()){
            arm.run_teleOp();

            arm.getTelemetry(mainTelemetry);
            mainTelemetry.update();
        }
    }
}
