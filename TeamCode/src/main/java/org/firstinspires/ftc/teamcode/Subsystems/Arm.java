    package org.firstinspires.ftc.teamcode.Subsystems;

    //import needed libraries
    import com.arcrobotics.ftclib.gamepad.GamepadEx;
    import com.arcrobotics.ftclib.gamepad.GamepadKeys;
    import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
    import com.arcrobotics.ftclib.hardware.motors.Motor;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import org.firstinspires.ftc.robotcore.external.Telemetry;

    //robot subsystem for extendable arm
    public class Arm {
        //motor objects & related variables
        Motor extendMotor;
        Motor armMotor;
        public static final double GEAR_RATIO = 0.3; // Output 60 Teeth, Input 20 Teeth
        public static final double ENCODER_RESOLUTION = 1425; //TODO switch to 2,786 when new motor is installed

        //absolute positions for arm in degrees
        private final int groundPose = 0;
        private final int basket1Pose = 230; //
        private final int basket2Pose = 230; //
        private final int specimenPickupPose = 95; //
        private final int rung1Pose = 95; //
        private final int rung2Pose = 130; //
        private final int  maxPose = 270;
        private double currentArmPose;
        private double currentEPose;
        private Arm_Poses armState;

        //arm
        public static int armTarget = 0;
        public double armPower;

        //extension
        public static int extendTarget = 0;

        //control variables
        GamepadEx driverOp;
        ToggleButtonReader y_button, a_button, x_button, b_button; //modes
        ToggleButtonReader d_up, d_down, d_left, d_right; //height toggles
        double leftY, rightY;

        //enum commands for arm positions
        public enum Arm_Poses {
            GROUND,
            BASKET1,
            BASKET2,
            SPECIMEN_PICKUP,
            RUNG1,
            RUNG2,
            DRIVER_CONTROL,
            MAX
        }

        //--------TELEOP COMMANDS---------
        public void init(HardwareMap hardwareMap, GamepadEx gamepad, boolean teleOp){
            driverOp = gamepad;

            //arm setup
            armMotor = new Motor(hardwareMap, "armMotor");
            extendMotor = new Motor(hardwareMap, "extensionMotor");
            armMotor.setInverted(true); //reverses the motor direction
            armMotor.encoder.setDirection(Motor.Direction.REVERSE); //makes encoder positive when pulled up
            armMotor.resetEncoder();
            armPower = 0.6;

            //TODO extension setup (will implement Aaron's Code)
            extendMotor.encoder.setDirection(Motor.Direction.REVERSE);
            extendMotor.resetEncoder();

            //set runModes based on teleOp vs Auto
            if(teleOp){
                armMotor.setRunMode(Motor.RunMode.RawPower);
                armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                armMotor.setInverted(true); //redundant but works lol
                extendMotor.setRunMode(Motor.RunMode.RawPower);
                extendMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            } else { //auto
                //arm init
                armMotor.setRunMode(Motor.RunMode.PositionControl);
                armMotor.setPositionCoefficient(0.05); //tuned value for position controller
                armMotor.setInverted(true);
                armMotor.setDistancePerPulse( (360 / ENCODER_RESOLUTION) * GEAR_RATIO); //approximately 0.0758
                armMotor.setTargetPosition(0);
                armMotor.set(0);

                //extension init
                extendMotor.setRunMode(Motor.RunMode.PositionControl);
                extendMotor.setDistancePerPulse(0.5); //TODO test different values for smooth
                extendMotor.setTargetDistance(0);
                extendMotor.set(0);
            }

            //---initialize toggles & buttons---
            d_up = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.DPAD_UP
            );
            d_down = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.DPAD_DOWN
            );
            d_left = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.DPAD_LEFT
            );
            d_right = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.DPAD_RIGHT
            );

            y_button = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.Y
            );
            a_button = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.A
            );
            x_button = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.X
            );
            b_button = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.B
            );

        }


        public void initExtend(HardwareMap hardwareMap){
            extendMotor = new Motor(hardwareMap, "extensionMotor");
            extendMotor.encoder.setDirection(Motor.Direction.REVERSE);
            extendMotor.resetEncoder();
            extendMotor.setRunMode(Motor.RunMode.RawPower);
            extendMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        public void runPassiveExtend(){
            extendMotor.set(-0.3);
        }

        private void updateToggles(){
            d_up.readValue();
            d_down.readValue();
            d_left.readValue();
            d_right.readValue();
            y_button.readValue();
            a_button.readValue();
            x_button.readValue();
            b_button.readValue();
        }


        //TODO
        /**
         *Competition-rated teleOp method with limits and shortcuts
         */
        public void run_teleOp(){
            //update variables
            currentArmPose = armMotor.getCurrentPosition();
            currentEPose = extendMotor.getCurrentPosition();
            leftY = driverOp.getLeftY();
            rightY = driverOp.getRightY();
            updateToggles();


            //arm control
            if(leftY > 0){
                armState = Arm_Poses.DRIVER_CONTROL;
                armMotor.set(armPower); //up
            } else if (leftY < 0){
                armState = Arm_Poses.DRIVER_CONTROL;
                armMotor.set(-armPower); //down
            } else {
                armMotor.set(0); //0 passive hold
            }

            //extension control
            if(rightY > 0){
                extendMotor.set(1);
            } else if(rightY < 0){
                extendMotor.set(-1);
            } else {
                extendMotor.set(0);
            }

            if(y_button.getState()){
                armPower = 1;
            } else {
                armPower = 0.6;
            }



        }

        //--------AUTO COMMANDS------------
        /**
         * main command to control arm
         * @param pose enum Arm State
         */
        public void setPose(Arm_Poses pose){
            switch(pose){
                case GROUND:
                    armTarget = groundPose;

                case BASKET1:
                    armTarget = basket1Pose;

                case BASKET2:
                    armTarget = basket2Pose;

                case SPECIMEN_PICKUP:
                    armTarget = specimenPickupPose;

                case RUNG1:
                    armTarget = rung1Pose;

                case RUNG2:
                    armTarget = rung2Pose;

                default:
                    armTarget = groundPose;
            }
            armMotor.setTargetDistance(armTarget);//highest of numbers

        }

        /**
         * Precise Auto Command to control the arm position (degrees)
         * Maximum Degrees is 100°
         * Minimum Degrees is 0°
         * @param degrees exact position of bot in degrees
         */
        public void setTarget(int degrees){
            if(degrees > maxPose){
                armTarget = maxPose;
            }  else armTarget = Math.max(degrees, groundPose);
            armMotor.setTargetDistance(armTarget);//highest of numbers
        }

        /**
         * An all in one command to control the arm (Pedro_TeleOp Only)
         * @param pose double value of arm angle
         */
        public void goTo(int pose){
            armTarget = pose;
            armMotor.setTargetDistance(armTarget);
            armMotor.set(0.3);
        }

        /**
         * Vital Arm Command to Update Telemetry Values and Run to Target
         */
        public void update(){
            currentArmPose = armMotor.getCurrentPosition();
            armMotor.set(0.3); //percentage of power to get to targets

            //TODO extension
        }

        //TODO
        public void extend(int pose){
            extendTarget = pose;
        }

        public void getTelemetryBRIEF(Telemetry telemetry){
            telemetry.addLine("----Arm Control Data----");
            telemetry.addData("Arm Pose:", armMotor.getCurrentPosition());
            telemetry.addData("Extend Pose: ", extendMotor.getCurrentPosition());

        }

        public void getTelemetryFULL(Telemetry telemetry){
            telemetry.addLine("----Arm Control Data----");
            telemetry.addData("Arm Pose:", armMotor.getCurrentPosition());
            telemetry.addData("Extend Pose: ", extendMotor.getCurrentPosition());
            telemetry.addData("Arm Target: ", armTarget);
            telemetry.addData("Arm Power: ", armMotor.get());
            telemetry.addData("Extend Target: ", extendTarget);


        }
    }