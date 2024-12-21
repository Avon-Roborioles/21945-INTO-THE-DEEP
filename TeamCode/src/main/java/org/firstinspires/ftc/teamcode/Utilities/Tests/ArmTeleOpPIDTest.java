package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@Config
@TeleOp(name="Arm PID TeleOp Test", group="Tests")
public class ArmTeleOpPIDTest extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Arm arm = new Arm();

    public static double newKp = 0;
    public static double newKi = 0;
    public static double newKd = 0;
    public static double maxintegralsum = 0;
    public static double stability = 0;
    public static double gain = 0;

    MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx driverOp = new GamepadEx(gamepad2);

        arm.initPID(hardwareMap,driverOp,true);


        waitForStart();

        while(opModeIsActive()){
            arm.setPID(newKp,newKi,newKd,maxintegralsum,stability,gain);
            arm.run_PIDTeleOp();
            arm.getTelemetryPID(mainTelemetry);
            mainTelemetry.update();
        }
    }
}
