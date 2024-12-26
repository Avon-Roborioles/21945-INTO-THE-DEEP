    package org.firstinspires.ftc.teamcode.Subsystems;

    //import needed libraries
    import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
    import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FullStateFeedback;
    import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
    import com.ThermalEquilibrium.homeostasis.Utils.Vector;
    import com.acmerobotics.roadrunner.profile.MotionProfile;
    import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
    import com.acmerobotics.roadrunner.profile.MotionState;
    import com.arcrobotics.ftclib.gamepad.GamepadEx;
    import com.arcrobotics.ftclib.gamepad.GamepadKeys;
    import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
    import com.arcrobotics.ftclib.hardware.motors.Motor;
    import com.arcrobotics.ftclib.hardware.motors.MotorEx;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.Telemetry;

    //robot subsystem for extendable arm
    public class Arm {
        //motor objects & related variables
        MotorEx armMotor;
        MotorEx extendMotor;
        public static final double GEAR_RATIO = 0.3; // Output 60 Teeth, Input 20 Teeth
        public static final double ENCODER_RESOLUTION = 1425; //TODO switch to 2,786 when new motor is installed

        //absolute positions
        private final int groundPose = 0;
        private final int autoGround = 970;
        private final int basketPose = 5600;
        private final int rung1Pose = 2000;
        private final int rung2Pose = 2700;
        private final int maxArmPose = 5750;


        private final int maxExtendPose = 2700;

        private int currentArmPose;
        private int currentExtendPose;
        private Arm_Modes armMode;

        //arm
        public double previousTarget = 0;
        public double instantTarget = 0;
        public double armTarget = 0;
        public double armPower = 0;

        //extension
        public int extendTarget = 0;
        public double extendPower = 0;

        //Motion Profile + Full State Feedback PID Controller (Arm) & Basic PID Controller (Extend)
        private final double kpArm = 0.002;
        private final double kpExtend = 0.01; //0.05
        private final double ka = 0.0004;
        private final double MAX_VELOCITY = 2000;
        private final double MAX_ACCELERATION = 4000;
        MotionProfile motionProfile;
        Vector armCoefficients;
        FullStateFeedback armController;
        PIDCoefficients extendCoefficients;
        BasicPID extendController;
        ElapsedTime time;
        boolean busy = false;

        //control variables
        GamepadEx driverOp;
        ToggleButtonReader y_button, a_button, x_button, b_button; //modes
        ToggleButtonReader d_up, d_down, d_left, d_right; //height toggles
        double leftY, rightY;

        //enum commands for arm modes
        public enum Arm_Modes {
            DRIVER_MODE,
            HOLD_MODE,
            HANG_MODE
        }
        public boolean extendHold = true;
        boolean hangPriority = false;

        //--------TELEOP COMMANDS---------
        public void init(HardwareMap hardwareMap, GamepadEx gamepad, boolean teleop){
            driverOp = gamepad;
            time = new ElapsedTime();

            armCoefficients = new Vector(new double[] {kpArm,ka});
            armController = new FullStateFeedback(armCoefficients);

            extendCoefficients = new PIDCoefficients(kpExtend,0,0);
            extendController = new BasicPID(extendCoefficients);

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

            //arm
            armMotor = new MotorEx(hardwareMap,"armMotor");
            armMotor.setInverted(true);
            armMotor.encoder.setDirection(Motor.Direction.REVERSE);
            armMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.stopAndResetEncoder();
            armMotor.setRunMode(Motor.RunMode.RawPower);

            motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(armMotor.getCurrentPosition(),0), new MotionState(armTarget,0), MAX_VELOCITY,MAX_ACCELERATION);


            //TODO - extend
            extendMotor = new MotorEx(hardwareMap,"extensionMotor");
            extendMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            extendMotor.stopAndResetEncoder();
            extendMotor.setRunMode(Motor.RunMode.RawPower);

        }

        public void initExtend(HardwareMap hardwareMap){
            extendMotor = new MotorEx(hardwareMap, "extensionMotor");
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

        public void run_teleOp(){
            //update variables
            currentArmPose = armMotor.getCurrentPosition();
            currentExtendPose = extendMotor.getCurrentPosition();
            leftY = driverOp.getLeftY(); //arm
            rightY = driverOp.getRightY(); //extend
            updateToggles();
            double decelTimeConstant = 0.06; //seconds to decelerate

            //manual arm control with limits
            //saves armTarget as arm current position for holding later
            if(leftY > 0){
                armMode = Arm_Modes.DRIVER_MODE;
                if(currentArmPose < maxArmPose) {
                    //int finalTarget = (int) (currentArmPose + ((-armMotor.getVelocity()) * decelTimeConstant));
                    setTarget(currentArmPose, currentExtendPose);
                    armPower = 0.7 * Math.abs(leftY); //added sensitivity
                }

            } else if(leftY < 0){
                armMode = Arm_Modes.DRIVER_MODE;
                if(currentArmPose > groundPose) {
                    //int finalTarget = (int) (currentArmPose + (armMotor.getVelocity() * decelTimeConstant));
                    setTarget(currentArmPose,currentExtendPose);
                    armPower = -0.5 * Math.abs(leftY); //added sensitivity
                }
            } else {
                armMode = Arm_Modes.HOLD_MODE;
            }

            //manual extension control with limits
            if (rightY < 0) {
                if(currentExtendPose < maxExtendPose) {
                    extendHold = false;
                    setTarget(currentArmPose,currentExtendPose);
                    extendPower = 1* Math.abs(rightY); //added sensitivity
                }

            }else if(rightY > 0){
                if(currentExtendPose > 0) {
                    extendHold = false;
                    setTarget(currentArmPose,currentExtendPose);
                    extendPower = -1* Math.abs(rightY); //added sensitivity
                }
            } else {
                extendHold = true;
            }


            //d-pad height presets
            if(d_left.wasJustPressed()){ //rung toggle
                armMode = Arm_Modes.HOLD_MODE;
                if(d_left.getState()){
                    setTarget(rung1Pose,maxExtendPose);
                } else {
                    setTarget(rung2Pose,maxExtendPose);
                }

            } else if(d_right.wasJustPressed()){
                armMode = Arm_Modes.HOLD_MODE;
                setTarget(autoGround,maxExtendPose);


            } else if(d_down.wasJustPressed()){ //ground
                armMode = Arm_Modes.HOLD_MODE;
                setTarget(groundPose,0);

            } else if(d_up.wasJustPressed()){ //basket
                armMode = Arm_Modes.HOLD_MODE;
                setTarget(basketPose,maxExtendPose);
            }

            //y button hang mode
            if(y_button.wasJustPressed()){
                armMode = Arm_Modes.HANG_MODE;
            }



            //control arm power for hang and hold modes
            if(armMode == Arm_Modes.HOLD_MODE){ //code from update() except motor set power
                MotionState state = motionProfile.get(time.time());

                double instantTarget = state.getX();
                double instantVelocity = state.getV();

                double measuredVelocity = armMotor.getVelocity() * -1;

                Vector measuredState = new Vector(new double[] {currentArmPose,measuredVelocity});
                Vector targetState = new Vector(new double[] {instantTarget,instantVelocity});

                try {
                    armPower = armController.calculate(targetState,measuredState);
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }

                //armMotor.setVelocity(-instantVelocity);
            } else if(armMode == Arm_Modes.HANG_MODE){
                armPower = -0.8; //near maximum power to pull up
            }

            //control extend power
            if(extendHold){
                extendPower = extendController.calculate(extendTarget,currentExtendPose);
            }

            armMotor.set(armPower);
            extendMotor.set(extendPower);
            updateToggles();
        }




        //--------AUTO COMMANDS------------
        public boolean ArmIsBusy(){
            return instantTarget != armTarget;
        }

        /**
         * Precise Auto Command to control the arm position (degrees)
         * Maximum Degrees is 100°
         * Minimum Degrees is 0°
         * @param arm position for arm
         * @param extend position for extension
         */
        public void setTarget(int arm, int extend){
            armTarget = Math.min(arm, maxArmPose);
            extendTarget = Math.min(extend,maxExtendPose);
            motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(armMotor.getCurrentPosition(),0), new MotionState(armTarget,0), MAX_VELOCITY,MAX_ACCELERATION);
            time.reset();
        }

        /**
         * Uses Trapezoidal Motion Profile + Full State Feedback to precisely control arm
         */
        public void update(){
            MotionState state = motionProfile.get(time.time());

            instantTarget = state.getX();
            double instantVelocity = state.getV();

            double measuredPosition = armMotor.getCurrentPosition();
            double measuredVelocity = armMotor.getVelocity() * -1;

            Vector measuredState = new Vector(new double[] {measuredPosition,measuredVelocity});
            Vector targetState = new Vector(new double[] {instantTarget,instantVelocity});

            try {
                armPower = armController.calculate(targetState,measuredState);
                extendPower = extendController.calculate(extendTarget,extendMotor.getCurrentPosition());
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            armMotor.setVelocity(-instantVelocity);
            armMotor.set(armPower);
            extendMotor.set(extendPower);
        }


        public void getTelemetry(Telemetry telemetry){
            telemetry.addLine("----ARM DATA----");
            telemetry.addData("Arm Mode: ", armMode);
            telemetry.addData("Arm Pose: ", armMotor.getCurrentPosition());
            telemetry.addData("Arm Target: ", armTarget);
            telemetry.addData("Arm Power: ", armPower);
            telemetry.addData("Extend Pose: ",extendMotor.getCurrentPosition());
            telemetry.addData("Extend Target: ", extendTarget);
            telemetry.addData("Extend Power: ", extendPower);
        }
    }