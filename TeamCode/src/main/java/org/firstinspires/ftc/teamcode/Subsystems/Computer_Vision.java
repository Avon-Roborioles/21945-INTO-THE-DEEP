package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;

/**
 * The Complete Class for our team's Computer Vision Efforts
 * Features include Specimen Alignment Calculations, Pit Sample Detection, & More
 * TODO - get offset of camera from ground and camera from bot center
 */
public class Computer_Vision {
    //LimeLight Pipeline Creation: (General) http://limelight.local:5801/ , (Windows) http://172.28.0.1:5801/ , (Mac) http://172.29.0.1:5801/
    //https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary

    /** Documentation Notes + Best Practices:

     Helpful Guides:

     Estimating Distance from Camera -> <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance">...</a>



     Documentation Notes
     One of the most important features limelight offers is the one-click crosshair. The crosshair, dual crosshair,
     tx, ty, ta, ts, tvert, and all other standard limelight NetworkTables readings will automatically latch to the
     contour you return from the python runPipeline() function.

     Limelight’s python scripting has access to the full OpenCV and numpy libraries.

     Beyond access to the image, the runPipeline() function also has access to data from the robot.
     FTC teams can use updatePythonInputs() and FRC teams can update the “llrobot” NetworkTables number array.
     Send any data from your robots to your python scripts for visualization or advanced applications
     (One might send IMU data, pose data, robot velocity, etc. for use in python scripts)

     The runPipeline function also outputs a number array that accessible with getPythonOutputs() and from
     the “llpython” networktables number array. This means you can bypass Limelight’s crosshair and other functionality
     entirely and send your own custom data back to your robots.
     * FTC Limelight Best Practices:
     * Download and backup all pipelines to your programming laptop.
     * Download a copy of the latest Limelight image to your programming laptop.
     * Record a list of your pipelines and their indices.
     * 1 - Dual Target Low
     * 2 - Dual Target High Cargo
     * Add strain reliefs to your Limelight's USB cable.
     * Consider hot-gluing all connections.
     */

    //useful variables
    Limelight3A limelight; //limelight camera object
    LLResult result; //limelight data from selected pipeline
    double[] pythonResults; //additional results from python pipelines
    LLStatus status;
    long resultAge; //tells us the age (in milliseconds of our results data)
    double targetX, targetY, targetArea; //easy to use values of target from pipeline
    double specimenAlignment; //array of calculated x, y, & heading values from limelight to adjust to hanged specimen
    double closestSample; //used when looking in the pit for a sample that matches our alliance color
    double allianceColor = 0; //0 is red, 1 is blue
    double targetDistance = 4;
    double strafeDistance = 0;
    double[] inputs = new double[2];

    public enum SampleColors{
        RED,
        BLUE,
        YELLOW
    }

    //llrobot array data - {int color}
    //llpython array data - {int strafeDistance}

    //-----------Private Vision Methods-------------------
    /**
     * Sends any Data to Custom LimeLight Camera Python Pipelines
     */
    private void sendData(){
        inputs[0] = allianceColor;
        inputs[1] = targetDistance;
        limelight.updatePythonInputs(inputs);
        //
    }

    /**
     * Pulls any Data from Custom Limelight Camera Python Pipelines
     */
    private double[] getData(){
        pythonResults = result.getPythonOutput();
        resultAge = result.getStaleness();
        return pythonResults; // Corrected: Now returns a double[]
    }

    //----------Common Auto Methods-----------------------
    /**
     * Used to setup cameras & vision pipelines
     * @param hardwareMap used to find camera objects
     */
    public void init(HardwareMap hardwareMap){
        //limelight camera initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); //set limelight data usage to 100 times per second
        limelight.start(); //starts the selected pipeline
        limelight.pipelineSwitch(2); //2nd index (pipeline 3) is updated vision script
        result = limelight.getLatestResult();

    }

    /**
     * Blue is 0, 1 is Red
     * @param blue search of blue color or red if false
     */
    public void setAllianceColor(boolean blue){
        if(blue){
            allianceColor = 0;
        } else {
            allianceColor = 1;
        }
    }

    /**
     * updates all current vision processes and pipelines
     */
    public void update(){
        //limelight.updatePythonInputs(new double[] {allianceColor});
        //getData();
        sendData(); //TODO used to push alliance color & target distance
        result = limelight.getLatestResult();
        if(result != null){
            if(result.isValid()){
                targetX = result.getTx();
                targetY = result.getTy();
                targetArea = result.getTa();
            }
        }
        status = limelight.getStatus();
    }


    /**
     * updates all current vision processes and pipelines + updates target Distance
     */
    public void update(double distance){
        targetDistance = distance;
        sendData(); //TODO used to push alliance color & target distance
        result = limelight.getLatestResult();
        if(result != null){
            if(result.isValid()){
                targetX = result.getTx();
                targetY = result.getTy();
                targetArea = result.getTa();
            }
        }
        status = limelight.getStatus();
    }

    /**
     * Backend Version of Vision Alignment
     */
    public double getAlignment(){
        //get values from limelight (ideally, all calculations are offloaded to camera)
        //limelight.pipelineSwitch(0);

        // Alternate Method to get Specimen Alignment on Java Side


        // Assuming targetX is already defined and represents the angle in radians
        //double targetX = 0.5; // Example value, replace with the actual value

        // Calculate the cotangent of the angle
        double cotangent = Math.cos(targetX) / Math.sin(targetX);

        // Define the distance 'd' (replace with the actual value or method to get d)
        double d = 4; // Example value

        // Calculate the strafe distance
        strafeDistance = d * cotangent;

        // Output the result
        //System.out.println("Strafe Distance: " + strafeDistance);
        return strafeDistance;
    }

    /**
     * Frontend Version of Vision Alignment
     */
    public double getMainAlignment(){
        return getData()[0];
    }

    //TODO
    public double getClosestSample(SampleColors color){
        //get values from limelight
        limelight.pipelineSwitch(1);

        return closestSample;
    }

    //TODO - uses field april tags to adjust bot pose on field
    public Pose getRealPose(){
        Pose result = new Pose(0,0,0);
        return result;
    }

    //-------------Match Recording & Misc-----------------

    //TODO - will record matches and save to USB in the future
    public void startVideo(){}

    //TODO
    public void pauseVideo(){}

    //TODO
    public void stopVideo(){}

    public void getTelemetry(Telemetry telemetry){
        telemetry.addLine("----Computer Vision Data----");
        telemetry.addData("Strafe Distance (Frontend): ", getMainAlignment());
        telemetry.addData("Strafe Distance (Backend): ", getAlignment());
        telemetry.addData("FPS: ", status.getFps());
        telemetry.addData("Temperature: ", status.getTemp());
        telemetry.addData("CPU Usage: ", status.getCpu());
        telemetry.addData("RAM Usage: ", status.getRam());
        telemetry.addData("Target X Degrees: ", targetX);
        telemetry.addData("Target Y Degrees: ", targetY);
        telemetry.addData("Target Area: ", targetArea);
    }
}
