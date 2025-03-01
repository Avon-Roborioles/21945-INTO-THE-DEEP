package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;

import java.util.List;

/**
 * Handles the team's Computer Vision operations using Limelight.
 * Features include:
 * - Specimen Alignment Calculations
 * - Pit Sample Detection
 * - Field AprilTag-based pose estimation
 *
 * Notes:
 * - Ensure proper calibration and backup of Limelight pipelines.
 * - Use network tables for communication between Python scripts and robot.
 */
public class Computer_Vision {

    // Limelight Camera Object
    private Limelight3A limelight;
    private LLResult result;
    private LLStatus status;
    private double[] pythonResults;
    private long resultAge;

    // Vision Data
    private double targetX, targetY, targetArea;
    private double specimenAlignment;
    private double closestSample;
    private double allianceColor = 0; // 0 = Red, 1 = Blue
    private double targetDistance = 4;
    private double strafeDistance = 0;
    private final double[] inputs = new double[2];

    public enum SampleColors {
        RED, BLUE, YELLOW
    }

    // ---------- Initialization & Setup ----------

    /**
     * Initializes the Limelight camera and sets up the vision pipeline.
     * @param hardwareMap HardwareMap to locate the Limelight camera.
     */
    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Process vision data at 100 Hz
        limelight.pipelineSwitch(2); // Load pipeline index 2
        limelight.start(); // Start the pipeline
        updateVisionData();
    }

    /**
     * Sets the alliance color for vision processing.
     * @param blue True if searching for blue, false for red.
     */
    public void setAllianceColor(boolean blue) {
        allianceColor = blue ? 0 : 1;
    }

    public void setTargetDistance(double distance){
        targetDistance = distance;
    }

    // ---------- Vision Data Processing ----------

    /**
     * Updates vision data and retrieves results from Limelight.
     */
    public void update() {
        sendData();
        updateVisionData();
    }

    /**
     * Updates vision data and sets a new target distance.
     * @param distance The updated target distance.
     */
    public void update(double distance) {
        targetDistance = distance;
        sendData();
        updateVisionData();
    }

    /**
     * Updates Limelight vision results.
     */
    private void updateVisionData() {
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if(true) {
                targetX = result.getTx();
                targetY = result.getTy();
                targetArea = result.getTa();
            }
        }
        status = limelight.getStatus();
    }

    /**
     * Sends relevant data to the Limelight Python pipeline.
     */
    private void sendData() {
        inputs[0] = allianceColor;
        inputs[1] = targetDistance;
        limelight.updatePythonInputs(inputs);
    }

    /**
     * Retrieves processed data from the Limelight Python pipeline.
     * @return Python-generated result array.
     */
    private double[] getData() {
        return result != null ? result.getPythonOutput() : new double[]{0};
    }

    // ---------- Vision-Based Alignments ----------

    /**
     * Calculates the required strafe distance to align with a target.
     * @return The computed strafe distance.
     */
    public double getAlignment() {
        double targetXRadians = Math.toRadians(targetX);
        return ((Math.cos(targetXRadians))/(Math.sin(targetXRadians))) * (targetDistance - 1.75);
        //return (targetDistance * Math.sin(Math.toRadians(90) - targetXRadians)) / Math.sin(targetXRadians);
    }

    /**
     * Retrieves alignment data from the Limelight pipeline.
     * @return The processed alignment value from Python scripts.
     */
    public double getMainAlignment() {
        return getData()[2]; //0
    }

    /**
     * Determines the closest sample of a specified color in the pit.
     * @param color The desired sample color.
     * @return Distance to the closest sample.
     */
    public double getClosestSample(SampleColors color) {
        limelight.stop();
        limelight.pipelineSwitch(2);
        limelight.start();
        return closestSample;
    }

    /**
     * Uses field AprilTags to estimate the robot's real-world pose.
     * @return The estimated Pose object.
     */
    public Pose getRealPose() {
        return new Pose(0, 0, 0); // Placeholder for future implementation
    }

    // ---------- Telemetry & Debugging ----------

    /**
     * Provides telemetry output for debugging.
     * @param telemetry The telemetry object to log data.
     */
    public void getTelemetry(Telemetry telemetry) {
        telemetry.addLine("---- Computer Vision Data ----");

        // Connection & Result Status
        telemetry.addData("Limelight Connected:", status != null);
        telemetry.addData("Result Valid:", result != null && result.isValid());
        telemetry.addData("Result Age (ms):", result != null ? result.getStaleness() : "N/A");

        // Target Data
        telemetry.addData("Target X (Degrees):", result.getTx());
        telemetry.addData("Target Y (Degrees):", result.getTy());
        telemetry.addData("Target Area:", result.getTa());

        // Limelight System Stats
        if (status != null) {
            telemetry.addData("FPS:", status.getFps());
            telemetry.addData("Temperature:", status.getTemp());
            telemetry.addData("CPU Usage:", status.getCpu());
            telemetry.addData("RAM Usage:", status.getRam());
        }

        // Alignment Calculations
        telemetry.addData("Strafe Distance (Frontend):", getMainAlignment());

        double txRad = Math.toRadians(targetX);
        telemetry.addData("Strafe Distance (Backend):", getAlignment());
    }

    // ---------- Match Recording (Future Features) ----------

    public void startVideo() { /* TODO: Implement match recording */ }
    public void pauseVideo() { /* TODO: Implement match recording */ }
    public void stopVideo() { /* TODO: Implement match recording */ }
}