    package org.firstinspires.ftc.teamcode.Subsystems;

    //import needed libraries
    import com.acmerobotics.dashboard.config.Config;
    import com.arcrobotics.ftclib.controller.PIDController;
    import com.arcrobotics.ftclib.gamepad.GamepadEx;
    import com.arcrobotics.ftclib.gamepad.GamepadKeys;
    import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
    import com.arcrobotics.ftclib.hardware.motors.Motor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import org.firstinspires.ftc.robotcore.external.Telemetry;


    //robot subsystem for extendable arm
    @Config
    public class Arm {
        //motor objects & related variables
        Motor extensionMotor;
        Motor armMotor;
        DcMotorEx test;
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
        private volatile boolean isMotorTimerRunning = false;
        private Thread motorTimerThread;

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
         * testing-rated Arm Command to initialize motors & other variables
         * @param hardwareMap needed to access robot config
         */
        public void initBASIC(HardwareMap hardwareMap, GamepadEx gamepad){
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


        public void initNEW(HardwareMap hardwareMap, GamepadEx gamepad, boolean teleOp){
            driverOp = gamepad;
            armMotor = new Motor(hardwareMap, "armMotor");
            extensionMotor = new Motor(hardwareMap, "extensionMotor");

            armMotor.setInverted(true);
            armMotor.encoder.setDirection(Motor.Direction.REVERSE);
            //armMotor.stopAndResetEncoder();
            armMotor.resetEncoder();

            //set runModes based on teleOp vs Auto
            if(teleOp){
                armMotor.setRunMode(Motor.RunMode.RawPower);
            } else {
                //auto
                armMotor.setRunMode(Motor.RunMode.PositionControl);
                armMotor.setPositionCoefficient(0.05); //MUST HAVE
                armMotor.setInverted(true);
                armMotor.setDistancePerPulse(0.015);
                armMotor.setTargetPosition(0);
                armMotor.set(0);
            }

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

            /* arm extension control V1 - Greatly affects Arm Control & Can't use well

            code below sucks causes backwards movement
            if(a_button.getState()) {
                extensionMotor.set(0.2);
            } else {
                extensionMotor.set(-1);
            } */

            /* CODE FOR NO MORE PID IN EXTENSION

            if(a_button.getState() && !isMotorTimerRunning) {
                // Start a new timer thread only if one isn't already running
                startMotorTimer(0.2); 
            } else if(!a_button.getState() && !isMotorTimerRunning) {
                // Start a new timer thread with -1 power
                startMotorTimer(-1.0);
            }*/

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

        /* USES THREADING FOR THE SLEEP METHOD BC JAVA STINKS AND HAS NO WAIT METHOD
        private void startMotorTimer(final double power) {
            if (isMotorTimerRunning) {
                return; // Don't start a new thread if one is already running
            }

            isMotorTimerRunning = true;
            motorTimerThread = new Thread(new Runnable() {
                public void run() {
                    try {
                        extensionMotor.set(power);
                        Thread.sleep(2000); // Sleep for 2 seconds
                        extensionMotor.set(0);
                    // THREADING IS WEIRD AND WILL THROW EXCEPTIONS
                    } catch (InterruptedException e) {
                        // Handle interruption when angy
                        extensionMotor.set(0);
                    } finally {
                        isMotorTimerRunning = false;
                    }
                }
            });
            motorTimerThread.start();
        }

         */

        /* KILL THREADS
        public void cleanup() {
            if (motorTimerThread != null && motorTimerThread.isAlive()) {
                motorTimerThread.interrupt();
            }
        }

         */

        //--------AUTO COMMANDS------------
        //TODO
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
            //armMotor.setTargetPosition(armTarget);


            armMotor.setTargetDistance(armTarget);
            armMotor.set(0.3);
        }


        public void extend(int pose){
            extendTarget = pose;
        }


        public void getTelemetryBRIEF(Telemetry telemetry){
            telemetry.addData("Arm Pose:", armMotor.getCurrentPosition());
            telemetry.addData("Arm Target: ", armTarget);
            telemetry.addData("Extend Pose: ", extensionMotor.getCurrentPosition());
            telemetry.addData("Extend Target: ", extendTarget);

        }

        public void getTelemetryFULL(Telemetry telemetry){
            telemetry.addLine("Arm & Extension Data");
            telemetry.addData("Arm Pose:", armMotor.getCurrentPosition());
            telemetry.addData("Extend Pose: ", extensionMotor.getCurrentPosition());
            telemetry.addData("Arm Target: ", armTarget);
            telemetry.addData("Arm Power: ", armMotor.get());
            telemetry.addData("Extend Target: ", extendTarget);


        }
    }
