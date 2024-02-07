package org.firstinspires.ftc.teamcode.Constants_and_Setpoints;

public class Constants {

    /**
     * !!!!!
     * BE CAREFUL WHEN CHANGING VALUES, IT WILL AFFECT ALL OF THE CODE
     * !!!!!!
     * */


    /**Odometry position*/

    public static double X = 0;
    public static double Y = 0;
    public static double Heading = 0;

    /**Drive PID's*/

    public static double driveP = 0.06;
    public static double driveD = 0.001;
    public static double driveF = 0;

    public static double strafeP = 0.04;
    public static double strafeD = 0.001;
    public static double strafeF = 0;

    public static double rotationP = 0.02;
    public static double rotationD = 0.001;
    public static double rotationF = 0;

    /**randomization position*/
    public static int propPos = 0;

    /**Odometry constants*/
    public static double botHeading;

    public static double realHeading;

    public static boolean collection_on = false;

    public static boolean drop_pixel_area = false;

    /**Pivot Pid values*/
    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001, pivot_f = 0.1;

    /**Slide Pid values*/
    public static double slide_p = 0.004, slide_i = 0, slide_d = 0.0001, slide_f = 0;

    /** pathing constants*/

    public static final double xStopDistanceAtMaxVelocity = 7;
    public static final double ystopDistanceAtMaxVelocity = 5;

    //in cm's per sec, need to get the real value of these
    public static double maxXVelocity = 65;
    public static double maxYVelocity = 43;

    public static final double scaleFactor = maxYVelocity/maxXVelocity;

    public static double velocityDecreasePerPoint = 2.5;

    //in cm's per sec, these need to be tuned
    public static final double maxXAcceleration = 20;
    public static final double maxYAcceleration =  maxXAcceleration*0.66;

    /**teleop driver constants*/
    public static double throttle = 0.6;

    public static double vertical;
    public static double horizontal;
    public static double pivot;

    public static double Vertical;
    public static double Horizontal;
    public static double Pivot;

    public static double robotRadius = 23;

}
