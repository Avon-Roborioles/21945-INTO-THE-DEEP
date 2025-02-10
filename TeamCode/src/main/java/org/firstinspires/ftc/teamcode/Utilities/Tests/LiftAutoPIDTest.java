package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;

import java.util.concurrent.TimeUnit;

@Autonomous
public class LiftAutoPIDTest extends AutoBase {
    MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public enum State{
        RETRIEVE,
        SCORE,
        FINISH
    }
    public State currentState;
    GamepadEx driverOp;
    boolean objectiveDone = false;
    int count = 0;

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
        mainTelemetry.addData("Count: ", count);
       getSubsystemTelemetry(mainTelemetry);
        mainTelemetry.addData("State: ", currentState);
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

        currentState = State.RETRIEVE;
        lift.setTarget(2200);

        while(opModeIsActive()){
            switch(currentState){
                case RETRIEVE:
                    if((!lift.IsBusy())){
                        waitSeconds(1.5);
                        lift.setTarget(1700);
                        currentState = State.SCORE;
                        break;
                    }
                case SCORE:
                    if(!lift.IsBusy()){
                        count++;
                        waitSeconds(1.5);

                        if(count == 5){
                            lift.setTarget(0);
                            currentState = State.FINISH;
                            break;
                        } else {
                            lift.setTarget(0);
                            currentState = State.RETRIEVE;
                        }
                    }
                case FINISH:
                    break;
            }

            updateAuto();
        }
    }
}
