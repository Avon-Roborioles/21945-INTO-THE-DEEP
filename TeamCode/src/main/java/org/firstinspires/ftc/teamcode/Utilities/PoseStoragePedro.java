package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Utilities.pedroPathing.localization.Pose;

//Same Functionality as other PoseStorage but for Pedro-Pathing
//having a new file for this makes merging easier on Git

public class PoseStoragePedro {
    //**Reminder that Pedro-Pathing's Origin is at the bottom left of the field
    //the maximum distance in each axis in 140" (inches)
    //adding +72" to each coordinate converts RR Poses to PP Poses

    public static Pose CurrentPose = null;

    //Left Side Poses
    public static Pose LeftStartPose = new Pose(-35+72,-60+72, Math.toRadians(90));

    


}
