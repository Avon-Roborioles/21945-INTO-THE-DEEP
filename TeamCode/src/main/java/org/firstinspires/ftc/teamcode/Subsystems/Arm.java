    package org.firstinspires.ftc.teamcode.Subsystems;

    //import needed libraries
    import com.arcrobotics.ftclib.gamepad.GamepadEx;
    import com.arcrobotics.ftclib.gamepad.GamepadKeys;
    import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
    import com.arcrobotics.ftclib.hardware.motors.Motor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import org.firstinspires.ftc.robotcore.external.Telemetry;


    //robot subsystem for extendable arm
    public class Arm {
        //motor objects & related variables
        Motor extendMotor;
        Motor armMotor;
        DcMotorEx test;
        public static final double GEAR_RATIO = 0.3; // Output 60 Teeth, Input 20 Teeth
        public static final double ENCODER_RESOLUTION = 1425; //TODO switch to 2,786 when new motor is installed

        //absolute positions for arm in degrees
        private final int groundPose = 0;
        private final int basket1Pose = 0; //TODO
        private final int basket2Pose = 0; //TODO
        private final int specimenPickupPose = 0; //TODO
        private final int rung1Pose = 0; //TODO
        private final int rung2Pose = 0; //TODO
        private final int  maxPose = 80;
        private double currentArmPose;
        private double currentEPose;
        private Arm_Poses armState;

        //arm
        public static int armTarget = 0;
        public double armPower;

        //extension
        public static int extendTarget = 0;
        private volatile boolean isMotorTimerRunning = false;
        private Thread motorTimerThread;

        //control variables
        GamepadEx driverOp;
        ToggleButtonReader a_button, d_up;
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

        /**
         * testing-rated Arm Command to initialize motors & other variables
         * @param hardwareMap needed to access robot config
         */
        public void initBASIC(HardwareMap hardwareMap, GamepadEx gamepad){
            extendMotor = new Motor(hardwareMap, "extensionMotor");
            armMotor = new Motor(hardwareMap, "armMotor");
            extendMotor.setRunMode(Motor.RunMode.RawPower);
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
            extendMotor.set(-1);
            currentArmPose = 0;

        }


        public void initNEW(HardwareMap hardwareMap, GamepadEx gamepad, boolean teleOp){
            driverOp = gamepad;

            //arm setup
            armMotor = new Motor(hardwareMap, "armMotor");
            extendMotor = new Motor(hardwareMap, "extensionMotor");
            armMotor.setInverted(true); //reverses the motor direction
            armMotor.encoder.setDirection(Motor.Direction.REVERSE); //makes encoder positive when pulled up
            armMotor.resetEncoder();

            //TODO extension setup (will implement Aaron's Code)

            //set runModes based on teleOp vs Auto
            if(teleOp){
                armMotor.setRunMode(Motor.RunMode.RawPower);
                armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                armMotor.setInverted(true); //redundant but works lol
                extendMotor.setRunMode(Motor.RunMode.RawPower);
                extendMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            } else { //auto
                //arm
                armMotor.setRunMode(Motor.RunMode.PositionControl);
                armMotor.setPositionCoefficient(0.05); //tuned value for position controller
                armMotor.setInverted(true);
                armMotor.setDistancePerPulse( (360 / ENCODER_RESOLUTION) * GEAR_RATIO); //approximately 0.0758
                armMotor.setTargetPosition(0);
                armMotor.set(0);

                //TODO extension
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
                extendMotor.set(0);
                extendMotor.resetEncoder();
                currentEPose = extendMotor.getCurrentPosition();

                // For arm motor, just update current position without resetting
                currentArmPose = armMotor.getCurrentPosition();
            } else {
                // Retract extension motor when not in setup mode
                extendMotor.set(-1);
            }

            d_up.readValue();
        }

        //TODO
        /**
         *Competition-rated teleOp method with limits and shortcuts
         */
        public void run_teleOp(){
            currentArmPose = armMotor.getCurrentPosition();
            currentEPose = extendMotor.getCurrentPosition();

            //get joystick readings
            leftY = driverOp.getLeftY();
            rightY = driverOp.getRightY();

            //arm control
            if(leftY > 0){
                armMotor.set(0.6); //up
            } else if (leftY < 0){
                armMotor.set(-0.6); //down
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


            a_button.readValue(); //update a_button toggle
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


            if(driverOp.gamepad.x){
                extendMotor.set(-1);
            } else {
                extendMotor.set(0);
            }

            if(driverOp.gamepad.b){
                extendMotor.set(1);
            } else {
                extendMotor.set(0);
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
         * An all in one command to control the arm (TeleOp Only)
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