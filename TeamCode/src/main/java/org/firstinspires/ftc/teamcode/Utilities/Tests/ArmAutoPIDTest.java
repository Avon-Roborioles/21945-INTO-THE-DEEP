package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;

import java.util.concurrent.TimeUnit;

@Autonomous
public class ArmAutoPIDTest extends AutoBase {
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
        arm.getTelemetry(mainTelemetry);
        intake.getTelemetryFULL(mainTelemetry);
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
        arm.setTarget(3000,0); //enough room for extend later
        intake.pickup();

        while(opModeIsActive()){
            switch(currentState){
                case RETRIEVE:
                    if((!arm.ArmIsBusy())){
                            waitSeconds(1.5);
                            arm.setTarget(5600, 2000);
                            currentState = State.SCORE;
                            break;
                    }
                case SCORE:
                    if(!arm.ArmIsBusy()){
                        count++;
                        intake.drop();
                        waitSeconds(1.5);

                        if(count == 5){
                            arm.setTarget(0, 0);
                            currentState = State.FINISH;
                            break;
                        } else {
                            arm.setTarget(900,2700);
                            currentState = State.RETRIEVE;
                            intake.pickup();
                        }
                    }
                case FINISH:
                    break;
            }

            updateAuto();
        }
    }
}
