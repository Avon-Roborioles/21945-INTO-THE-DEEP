package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.Autonomous.Sample_Auto;
import org.firstinspires.ftc.teamcode.Utilities.PoseStoragePedro;

import java.util.concurrent.TimeUnit;

@Autonomous
public class ArmAutoPIDTest extends AutoBase {
    MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public enum State{
        RUNG,
        BUCKET,
        GROUND
    }
    public State currentState;
    GamepadEx driverOp;
    boolean objectiveDone = false;

    public void waitSeconds(double seconds){
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(seconds >= time.time(TimeUnit.SECONDS)){
            updateAuto();
            if(isStopRequested()){
                break;
            }
            if(!opModeIsActive()){
                break;
            }
        }
    }

    public void updateAuto(){
        subsystemsUpdate();
        arm.getTelemetryPID(mainTelemetry);
        intake.getTelemetryFULL(mainTelemetry);
        mainTelemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //init
        driverOp = new GamepadEx(gamepad1);
        init_classes(driverOp);

        while(opModeInInit()){
            updateAuto();
        }

        waitForStart();

        currentState = State.RUNG;
        arm.setTarget(1000,0);

        while(opModeIsActive()){
            switch(currentState){
                case RUNG:
                    if(!arm.ArmIsBusy()){
                        waitSeconds(3);
                        arm.setTarget(5600,2000);
                        currentState = State.BUCKET;
                        break;
                    }
                case BUCKET:
                    if(!arm.ArmIsBusy()){
                        waitSeconds(3);

                        arm.setTarget(0,0);
                        currentState = State.GROUND;
                        break;
                    }
                case GROUND:
                    break;
            }

            updateAuto();
        }
    }
}
