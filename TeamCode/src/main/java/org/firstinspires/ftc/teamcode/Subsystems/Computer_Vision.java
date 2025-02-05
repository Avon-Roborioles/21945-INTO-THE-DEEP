package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    int allianceColor = 0; //0 is red, 1 is blue

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
     * @param values sent to limelight python pipelines as input
     */
    private void sendData(double[] values){
        limelight.updatePythonInputs(values);
        //
    }

    /**
     * Pulls any Data from Custom Limelight Camera Python Pipelines
     */
    private void getData(){
        pythonResults = result.getPythonOutput();
        resultAge = result.getStaleness();

        //return pythonResults;
    }

    //----------Common Auto Methods-----------------------
    /**
     * Used to setup cameras & vision pipelines
     * @param hardwareMap used to find camera objects
     */
    public void init(HardwareMap hardwareMap, boolean redAlliance){
        //limelight camera initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); //set limelight data usage to 100 times per second
        limelight.pipelineSwitch(0); //selects the 1st pipeline saved on the camera (10 saved max)
        limelight.start(); //starts the selected pipeline

        //alliance selection
        if(redAlliance){
            allianceColor = 0;
        } else {
            allianceColor = 1;
        }
    }

    /**
     * updates all current vision processes and pipelines
     */
    public void update(){
        limelight.updatePythonInputs(new double[] {allianceColor});
        getData();
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


    //TODO
    /**
     * @return An array of alignment values for Pedro-Pathing (x, y, heading)
     */
    public double getSpecimenAlignment(){
        //get values from limelight (ideally, all calculations are offloaded to camera)
        limelight.pipelineSwitch(0);
        double angle_in_radians = Math.toRadians(targetX);
        double cotangent = Math.cos(angle_in_radians) / Math.sin(angle_in_radians);
        return 6 * cotangent;

    }

    //TODO
    public double getClosestSample(SampleColors color){
        //get values from limelight
        limelight.pipelineSwitch(1);

        return closestSample;
    }

    /**
     *
     * @return
     */
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
        telemetry.addData("FPS: ", status.getFps());
        telemetry.addData("Temperature: ", status.getTemp());
        telemetry.addData("CPU Usage: ", status.getCpu());
        telemetry.addData("RAM Usage: ", status.getRam());
        telemetry.addData("Target X Degrees: ", targetX);
        telemetry.addData("Target Y Degrees: ", targetY);
        telemetry.addData("Target Area: ", targetArea);
    }
}
