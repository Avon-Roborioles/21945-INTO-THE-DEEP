package org.firstinspires.ftc.teamcode.Utilities;


import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;

import java.util.ArrayList;

//an innovative feature that creates specified regions on the field for object detection, speed ups, etc
public class BoundedArea {

    private double xMin, yMin, xMax, yMax;
    private AreaType type;
    private ArrayList<Runnable> actions = new ArrayList<>();

    public enum AreaType {
        REGION,
        OBSTACLE
    }

    /**
     * Creates a specified region on the field
     * @param xMin Minimum X-Coordinate of the area
     * @param yMin Minimum Y-Coordinate of the area
     * @param xMax Maximum X-Coordinate of the area
     * @param yMax Maximum Y-Coordinate of the area
     */
    public BoundedArea(double xMin, double yMin, double xMax, double yMax){
        this.xMin = xMin;
        this.yMin = yMin;
        this.xMax = xMax;
        this.yMax = yMax;
    }

    public BoundedArea(double xMin, double yMin, double xMax, double yMax, AreaType type){
        this.xMin = xMin;
        this.yMin = yMin;
        this.xMax = xMax;
        this.yMax = yMax;
        this.type = type;
    }

    /**
     * Add a task to run when bot is in bounded Region
     * @param runnable add the task in a runnable like ()->{}
     */
    public void addTask(Runnable runnable){
        actions.add(runnable);
    }

    /**
     * Returns true or false if pose is within Bounded Area
     * @param pose The Pose Coordinate to Compare to
     * @return
     */
    public boolean comparePose(Pose pose){
        double x = pose.getX();
        double y = pose.getY();
        return (x >= xMin && x <= xMax) && (y >= yMin && y <= yMax);
    }


    /**
     * A passive method to perform assigned actions when within Bounded Area
     * @param pose The Current Pose of the bot to watch for
     */
    public void watch(Pose pose){
        if(comparePose(pose)) {
            for (Runnable action : actions) {
                action.run();
            }
        }
    }
}
