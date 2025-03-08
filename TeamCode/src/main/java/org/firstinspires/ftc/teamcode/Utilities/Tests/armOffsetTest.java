package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

//a simple program to test armOffset logic from auto to teleOp
@TeleOp(name="Arm Offset Test", group = "Tests")
public class armOffsetTest extends AutoBase {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    public void runOpMode () throws InterruptedException {


        GamepadEx driverOp =  new GamepadEx(gamepad1);
        init_classes(driverOp);

        while(opModeInInit()){
            runMenu(mainTelemetry);
            AutoPose = getAutoPose();
            PoseStorage.allianceSide = getAutoPose();
            mainTelemetry.update();
        }

        waitForStart();


        while(opModeIsActive()){
            PoseStorage.armOffset = arm.getPosition();
            PoseStorage.extendOffset = arm.getExtendPosition();
            getSubsystemTelemetry(mainTelemetry);
            mainTelemetry.update();
        }
    }
}
