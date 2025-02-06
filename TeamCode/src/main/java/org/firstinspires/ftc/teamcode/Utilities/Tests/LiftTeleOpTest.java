package org.firstinspires.ftc.teamcode.Utilities.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LiftTeleOpTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx driverOp = new GamepadEx(gamepad2);
        MotorEx liftMotor = new MotorEx(hardwareMap,"liftMotor");
        double rightY = driverOp.getRightY();

        MultipleTelemetry mainTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        liftMotor.setRunMode(Motor.RunMode.RawPower);
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //liftMotor.setInverted(true);
        //liftMotor.encoder.setDirection(Motor.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive()){
            if(rightY > 0){
                liftMotor.set(0.8 * rightY);
            } else if(rightY < 0){
                liftMotor.set(-0.5 * Math.abs(rightY));
            } else {
                liftMotor.set(0);
            }

            mainTelemetry.addData("Lift Pose: ", liftMotor.getCurrentPosition());
            mainTelemetry.addData("Lift Velocity: ", liftMotor.getVelocity());
            mainTelemetry.update();
        }
    }
}
