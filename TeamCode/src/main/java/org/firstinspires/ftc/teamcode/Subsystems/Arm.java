    package org.firstinspires.ftc.teamcode.Subsystems;

    //import needed libraries
    import com.acmerobotics.dashboard.config.Config;
    import com.arcrobotics.ftclib.controller.PIDController;
    import com.arcrobotics.ftclib.gamepad.GamepadEx;
    import com.arcrobotics.ftclib.gamepad.GamepadKeys;
    import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
    import com.arcrobotics.ftclib.hardware.motors.Motor;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import org.firstinspires.ftc.robotcore.external.Telemetry;


    //robot subsystem for extendable arm
    @Config
    public class Arm {
        //motor objects & related variables
        Motor extensionMotor;
        Motor armMotor;
        public static final double GEAR_RATIO = 3.0; // Output 60 Teeth, Input 20 Teeth
        private static final  double ticks_in_degree = 700 / 180.0;

        //absolute positions ("final" means they can't change later in code)
        private final double groundPose = 0; //TODO
        private final double basket1Pose = 0; //TODO
        private final double basket2Pose = 0; //TODO
        private final double maxPose = 0; //TODO
        private double currentArmPose;
        private double currentEPose;
        private Arm_Poses armState;

        //TODO - Tune these PID variables
        //arm
        PIDController armPIDController;
        public static  double arm_p = 0; //TODO change to final after tuning
        public static  double arm_i = 0;
        public static  double arm_d = 0;
        public static  double arm_f = 0;
        public static int armTarget = 0;
        public double armPower;

        //extension
        PIDController EPIDController;
        public static final double extend_p = 0;
        public static final double extend_i = 0;
        public static final double extend_d = 0;
        public static final double extend_f = 0;
        public static int extendTarget = 0;

        //control variables
        GamepadEx driverOp;
        ToggleButtonReader a_button, d_up;
        double leftY;

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

        /**
         * Default Arm Command to initialize motors & other variables
         * @param hardwareMap needed to access robot config
         */
        public void init(HardwareMap hardwareMap, GamepadEx gamepad){
            extensionMotor = new Motor(hardwareMap, "extensionMotor");
            armMotor = new Motor(hardwareMap, "armMotor");
            extensionMotor.setRunMode(Motor.RunMode.RawPower);
            armMotor.setRunMode(Motor.RunMode.RawPower);

            //gamepad variables
            driverOp = gamepad;

            //extensionMotor toggle
            a_button = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.A
            );

            //button to set extensionMotor to 0
            d_up = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.DPAD_UP
            );

            //start running EMotor & set ArmPose to 0
            extensionMotor.set(-1);
            currentArmPose = 0;
        }


        /**
         * Comp-Ready Arm Command to initialize motors WITH PID Control
         * @param hardwareMap needed to access robot config
         */
        public void initPID(HardwareMap hardwareMap, GamepadEx gamepad){
            driverOp = gamepad;

            armPIDController = new PIDController(arm_p, arm_i, arm_d);
            EPIDController = new PIDController(extend_p,extend_i, extend_d);

            //telemetry
            extensionMotor = new Motor(hardwareMap, "extensionMotor");
            armMotor = new Motor(hardwareMap, "armMotor");
            //armMotor.setInverted(true);
            armMotor.stopAndResetEncoder();

            extensionMotor.setRunMode(Motor.RunMode.RawPower);
            armMotor.setRunMode(Motor.RunMode.RawPower);

            armMotor.stopAndResetEncoder();

            a_button = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.A
            );

            //button to set extensionMotor to 0
            d_up = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.DPAD_UP
            );
        }

        public void initPoseControl(HardwareMap hardwareMap, GamepadEx gamepad){
            driverOp = gamepad;
            extensionMotor = new Motor(hardwareMap, "extensionMotor");
            armMotor = new Motor(hardwareMap, "armMotor");

            armMotor.stopAndResetEncoder();
            armMotor.setRunMode(Motor.RunMode.PositionControl);
            armMotor.setInverted(true);
            armMotor.setTargetPosition(0);

            a_button = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.A
            );

            //button to set extensionMotor to 0
            d_up = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.DPAD_UP
            );
        }

        /**
         * Helps pull in the extension Arm & set the position to 0
         */
        public void setupEMotor() {
            if(d_up.getState()) {
                // Stop extension motor and reset its position
                extensionMotor.set(0);
                extensionMotor.resetEncoder();
                currentEPose = extensionMotor.getCurrentPosition();

                // For arm motor, just update current position without resetting
                currentArmPose = armMotor.getCurrentPosition();
            } else {
                // Retract extension motor when not in setup mode
                extensionMotor.set(-1);
            }

            d_up.readValue();
        }

        /**
         * competition-rated method using a PID Controller for precise movement
         * @param driverOp needed to control arm
         */
        public void run_teleOp(GamepadEx driverOp){

        }

        /**
         * testing-rated method using raw power values for movement
         */
        public void run_teleOpBASIC(){
    //        currentArmPose = armMotor.getCurrentPosition();
    //        currentEPose = armMotor.getCurrentPosition();

            //update leftY joystick reading
            leftY = driverOp.getLeftY();

            if(leftY > 0){
                armMotor.set(-0.6);
            } else if (leftY < 0){
                armMotor.set(0.6);
            } else {
                armMotor.set(-0.05); //0 passive hold
            }

            //arm extension control V1 - Greatly affects Arm Control & Can't use well
    //        if(a_button.getState()) {
    //            extensionMotor.set(0.2);
    //        } else {
    //            extensionMotor.set(-1);
    //        }

            if(driverOp.gamepad.x){
                extensionMotor.set(-1);
            } else {
                extensionMotor.set(0);
            }

            if(driverOp.gamepad.b){
                extensionMotor.set(1);
            } else {
                extensionMotor.set(0);
            }


            a_button.readValue(); //update a_button toggle
        }

        //--------AUTO COMMANDS------------

        /**
         * main command to control arm
         * @param pose enum Arm State
         */
        public void set_pose(Arm_Poses pose){

        }

        /**
         * precise command to control arm
         * @param pose double value of arm position
         */
        public void set_pose(int pose){
            armTarget = pose;
        }

        public void set_pose_Main(int pose){
            armTarget = pose;
            armMotor.setTargetPosition(armTarget);
            armMotor.set(0.1); //change power to make movement smooth
        }


        public void extend(int pose){
            extendTarget = pose;
        }


        /**
         * update PID controller for arm in auto
         */
        public void update(){
            currentArmPose = armMotor.getCurrentPosition();
            currentEPose = extensionMotor.getCurrentPosition();

            //arm PID control
            double arm_pid = armPIDController.calculate(currentArmPose, armTarget);
            double arm_ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree) * arm_f);
            armPower = arm_pid + arm_ff;
            armMotor.set(armPower);

    //        double extend_pid = EPIDController.calculate(currentEPose, extendTarget);
    //        double extend_ff = Math.cos(Math.toRadians(extendTarget / ticks_in_degree) * extend_f);
    //        double extendPower = extend_pid + extend_ff;
    //        extensionMotor.set(extendPower);
        }


        public void getTelemetryBRIEF(Telemetry telemetry){
    //        telemetry.addData("Arm Pose:", currentArmPose);

        }

        public void getTelemetryFULL(Telemetry telemetry){
            telemetry.addData("Arm Pose:", armMotor.getCurrentPosition());
            telemetry.addData("Extend Pose: ", extensionMotor.getCurrentPosition());
            telemetry.addData("Arm Target: ", armTarget);
            telemetry.addData("Extend Target: ", extendTarget);


        }
    }
