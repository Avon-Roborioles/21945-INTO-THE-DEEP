package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@TeleOp(name="Bounded Area Test", group="Tests")
public class BoundedAreaTest extends LinearOpMode {
    private final org.firstinspires.ftc.teamcode.Subsystems.Drivetrain drivetrain = new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx driverOp = new GamepadEx(gamepad1);
        drivetrain.init(hardwareMap,driverOp, PoseStorage.RightStartPose);

        MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()){
            drivetrain.run_teleOp2();
            drivetrain.getTelemetry2(mainTelemetry);
            mainTelemetry.update();

        }
    }
}
