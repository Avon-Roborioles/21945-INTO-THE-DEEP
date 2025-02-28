package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Computer_Vision;

@TeleOp
public class EasyVisionTest extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Subsystems.Computer_Vision vision = new Computer_Vision();

    @Override
    public void runOpMode() throws InterruptedException {

        MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        vision.init(hardwareMap);


        while(opModeIsActive()){
            vision.update();

            vision.getTelemetry(mainTelemetry);

            mainTelemetry.update();

        }
    }
}
