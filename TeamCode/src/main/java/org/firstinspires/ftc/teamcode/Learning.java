package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Learning extends LinearOpMode {
    Servo servo = null;
    @Override
    public void runOpMode() throws InterruptedException{

        telemetry.addData( "This will show up once you press init",  5);
        telemetry.update();

        waitForStart();
        while(!isStopRequested()) {

            //calculates how much to change pos by
            double servoOffset = gamepad1.left_stick_y*.001;

            //gets current servo position
            double servoPos = servo.getPosition();
            //calculates next servo position
            servoPos = servoOffset + servoPos;


            //adds bounds to position
            if(servoPos< 0){
                servoPos = 0;
            } else if(servoPos > 1) {
                servoPos = 1;
            }

            //sets the postion of the servo
            servo.setPosition(servoPos);


            telemetry.addData("Servo Position", servo.getPosition());
    }
}
}




