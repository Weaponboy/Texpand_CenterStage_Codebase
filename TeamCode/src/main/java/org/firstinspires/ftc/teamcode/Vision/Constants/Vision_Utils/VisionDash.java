package org.firstinspires.ftc.teamcode.Vision.Constants.Vision_Utils;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;

@Config
public class VisionDash {

    public static Point target = new Point(0, 0);

    public static boolean debugMode;

    public static int blur = 75;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static double area_cutoff = 500;


//    public static int red_min_Y = 110;
//    public static int red_min_Cr = 180;
//    public static int red_min_Cb = 105;
//
//    public static int red_max_Y = 170;
//    public static int red_max_Cr = 230;
//    public static int red_max_Cb = 130;
//
//    public static int blue_min_Y = 110;
//    public static int blue_min_Cr = 180;
//    public static int blue_min_Cb = 105;
//
//    public static int blue_max_Y = 170;
//    public static int blue_max_Cr = 230;
//    public static int blue_max_Cb = 130;

    //Pole width
    public static int High_pole_min_width = 100;
    public static int High_pole_max_width = 500;
    public static int Med_pole_min_width = 70;
    public static int Med_pole_max_width = 125;

    public static int Low_pole_min_width = 100;
    public static int Low_pole_max_width = 155;

    //Cone color
    public static int pole_min_H = 15;
    public static int pole_min_S = 100;
    public static int pole_min_V = 60;

    public static int pole_max_H = 40;
    public static int pole_max_S = 255;
    public static int pole_max_V = 255;

    public static int cone_Min_H = 110;
    public static int cone_Min_S = 80;
    public static int cone_Min_V = 60;
    public static int cone_Max_H = 132;
    public static int cone_Max_S = 255;
    public static int cone_Max_V = 255;

    public static int PoleVisionEX = 40;

    public static int Red_cone_Min_H = 120;
    public static int Red_cone_Min_S = 40;
    public static int Red_cone_Min_V = 70;
    public static int Red_cone_Max_H = 200;
    public static int Red_cone_Max_S = 255;
    public static int Red_cone_Max_V = 255;

    public static int max_Cb = 255;
    public static int max_Cr = 255;
    public static int max_Y = 38;

    public static int min_Cb = 100;
    public static int min_Cr = 40;
    public static int min_Y = 10;


//    public static int max_Cb = 100;
//    public static int max_Cr = 165;
//    public static int max_Y = 255;
//
//    public static int min_Cb = 70;
//    public static int min_Cr = 135;
//    public static int min_Y = 100;


    public static double blackMultiplier = 2;



    public static double[] YCbCrReadout = {0, 0, 0};

}
