    package org.firstinspires.ftc.teamcode.Subsystems;

    //import needed libraries
    import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
    import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
    import com.arcrobotics.ftclib.gamepad.GamepadEx;
    import com.arcrobotics.ftclib.gamepad.GamepadKeys;
    import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
    import com.arcrobotics.ftclib.hardware.motors.Motor;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import org.firstinspires.ftc.robotcore.external.Telemetry;

    //robot subsystem for extendable arm
    public class Arm {
        //motor objects & related variables
        Motor extendMotor;
        Motor armMotor;
        DcMotorEx eMotor;
        DcMotorEx aMotor;
        public static final double GEAR_RATIO = 0.3; // Output 60 Teeth, Input 20 Teeth
        public static final double ENCODER_RESOLUTION = 1425; //TODO switch to 2,786 when new motor is installed

        //absolute positions for arm in degrees
        private final int groundPose = 0;
        private final int basket1Pose = 230; //
        private final int basket2Pose = 230; //
        private final int specimenPickupPose = 95; //
        private final int rung1Pose = 95; //
        private final int rung2Pose = 130; //
        private final int  maxPose = 3000;
        private final int extendMAX = 3000;
        private double currentArmPose;
        private double currentEPose;
        private Arm_Modes armMode;

        //arm
        public double armTarget = 0;
        public double armPower = 0;

        //extension
        public int extendTarget = 0;
        public double extendPower = 0;

        //pid control TODO - tune
        double kp = 0;
        double ki = 0;
        double kd = 0;
        double maximumIntegralSum = 0;
        double stabilityThreshold = 0;
        double lowPassGain = 0;
        PIDCoefficientsEx armPIDCoefficients = new PIDCoefficientsEx(kp,ki,kd,maximumIntegralSum,stabilityThreshold,lowPassGain);
        PIDEx armController = new PIDEx(armPIDCoefficients);

        //control variables
        GamepadEx driverOp;
        ToggleButtonReader y_button, a_button, x_button, b_button; //modes
        ToggleButtonReader d_up, d_down, d_left, d_right; //height toggles
        double leftY, rightY;

        //enum commands for arm modes
        public enum Arm_Modes {
            DRIVER_CONTROL,
            PRESET_MODE,
            HANG_MODE
        }

        //--------TELEOP COMMANDS---------
        public void setPID(double p, double i, double d, double sum, double stability, double gain){
            kp = p;
            ki = i;
            kd = d;
            maximumIntegralSum = sum;
            stabilityThreshold = stability;
            lowPassGain = gain;
            armPIDCoefficients = new PIDCoefficientsEx(kp,ki,kd,maximumIntegralSum,stabilityThreshold,lowPassGain);
        }

        public void init(HardwareMap hardwareMap, GamepadEx gamepad, boolean teleOp){
            driverOp = gamepad;

            //arm setup
            armMotor = new Motor(hardwareMap, "armMotor");
            extendMotor = new Motor(hardwareMap, "extensionMotor");
            armMotor.setInverted(true); //reverses the motor direction
            armMotor.encoder.setDirection(Motor.Direction.REVERSE); //makes encoder positive when pulled up
            armMotor.resetEncoder();
            armPower = 0.6;

            //extendMotor.encoder.setDirection(Motor.Direction.REVERSE);
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

        public void initPID(HardwareMap hardwareMap, GamepadEx gamepad, boolean teleop){
            driverOp = gamepad;
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

            aMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
            aMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            aMotor.setPower(0);

            eMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
            //eMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            //eMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            eMotor.setTargetPosition(0);
            eMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            eMotor.setPower(0);

//            armMotor = new Motor(hardwareMap, "armMotor");
//            armMotor.setInverted(true); //reverses the motor direction
//            armMotor.encoder.setDirection(Motor.Direction.REVERSE); //makes encoder positive when pulled up
//            armMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            armMotor.setRunMode(Motor.RunMode.RawPower);
//            armMotor.resetEncoder();
//
//            extendMotor = new Motor(hardwareMap, "extensionMotor");
//            extendMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            extendMotor.resetEncoder();
////            extendMotor.encoder.setDirection(Motor.Direction.REVERSE);
//            extendMotor.setInverted(true);
//            extendMotor.setRunMode(Motor.RunMode.PositionControl);
//            extendMotor.setDistancePerPulse(0.003260869565); //in - 7.5in/2300 ticks represents max range
//            extendMotor.setTargetDistance(0);
//            extendMotor.set(0);
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
                armMotor.set(armPower); //up
            } else if (leftY < 0){
                armMotor.set(-armPower); //down
            } else {
                armMotor.set(0); //0 passive hold
            }

            //extension control
            if(rightY > 0){
                extendMotor.set(-1);
            } else if(rightY < 0){
                extendMotor.set(1);
            } else {
                extendMotor.set(0);
            }

            if(y_button.getState()){
                armPower = 1;
            } else {
                armPower = 0.6;
            }



        }


        public void run_PIDTeleOp(){
            //update variables
            currentArmPose = aMotor.getCurrentPosition();
            currentEPose = eMotor.getCurrentPosition();
            leftY = driverOp.getLeftY();
            rightY = driverOp.getRightY();
            updateToggles();

            //manual arm control with limits
            if(leftY > 0){
                armMode = Arm_Modes.DRIVER_CONTROL;
                armTarget += 1;
            } else if(leftY < 0){
                armMode = Arm_Modes.DRIVER_CONTROL;
                armTarget -= 1;
            }

            //manual extension control with limits
            if (rightY < 0) {
                if(currentEPose < 2900) {
                    armMode = Arm_Modes.DRIVER_CONTROL;
                    extendTarget += 7;
                }

            }else if(rightY > 0){
                if(currentEPose > 0) {
                    armMode = Arm_Modes.DRIVER_CONTROL;
                    extendTarget -= 7;
                }
            }


            //d-pad height presets
            if(d_left.wasJustPressed()){
                armMode = Arm_Modes.PRESET_MODE;
                if(d_left.getState()){
                    armTarget = rung1Pose;
                } else {
                    armTarget = rung2Pose;
                }

            } else if(d_right.wasJustPressed()){
                armMode = Arm_Modes.PRESET_MODE;
                armTarget = specimenPickupPose;

            } else if(d_down.wasJustPressed()){
                armMode = Arm_Modes.PRESET_MODE;
                armTarget = groundPose;

            } else if(d_up.wasJustPressed()){
                armMode = Arm_Modes.PRESET_MODE;
                armTarget = groundPose;
                if(d_up.getState()){
                    armTarget = basket1Pose;
                } else {
                    armTarget = basket2Pose;
                }
            }

            //y button hang mode
            if(y_button.wasJustPressed()){
                armMode = Arm_Modes.HANG_MODE;
            }


            //arm + extension movements
            if(armMode == Arm_Modes.HANG_MODE){

            } else {
               //PID

            }
            eMotor.setTargetPosition(extendTarget);
            eMotor.setPower(1);
            updateToggles();
        }


        //--------AUTO COMMANDS------------
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
         * Vital Arm Command to Update Telemetry Values and Run to Target
         */
        public void update(){
            currentArmPose = armMotor.getCurrentPosition();
            armMotor.set(0.3); //percentage of power to get to targets

            //TODO extension
        }


        public void getTelemetryPID(Telemetry telemetry){
            telemetry.addData("Kp: ", kp);
            telemetry.addData("Ki: ", ki);
            telemetry.addData("Kd: ", kd);
            telemetry.addData("MaxIntegralSum: ", maximumIntegralSum);
            telemetry.addData("StabilityThreashold: ", stabilityThreshold);
            telemetry.addData("Low Pass Gain: ", lowPassGain);
            telemetry.addData("Arm Target: ", armTarget);
            telemetry.addData("Arm Pose: ", currentArmPose);
            telemetry.addData("Extend Target: ", eMotor.getTargetPosition());
            telemetry.addData("Extend Pose: ", currentEPose);
            telemetry.addData("Arm Mode: ", armMode);
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