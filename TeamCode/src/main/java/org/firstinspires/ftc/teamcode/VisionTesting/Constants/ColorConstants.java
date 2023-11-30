package org.firstinspires.ftc.teamcode.VisionTesting.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ColorConstants {

    public static int erode_const = 5;
    public static int dilate_const = 5;

    // Green color HSV values
    public static final double GREEN_LOWER_H = 65;
    public static final double GREEN_LOWER_S = 100;
    public static final double GREEN_LOWER_V = 0;

    public static final double GREEN_UPPER_H = 95;
    public static final double GREEN_UPPER_S = 400;
    public static final double GREEN_UPPER_V = 400;

    // Green color camera gain and exposure
    public static final double GREEN_CAMERA_GAIN = 30;  // Range: 0-255
    public static final double GREEN_CAMERA_EXPOSURE = 10;  // Range: 0-10000

    // White color HSV values
    public static final double WHITE_LOWER_H = 0;
    public static final double WHITE_LOWER_S = 0;
    public static final double WHITE_LOWER_V = 220;

    public static final double WHITE_UPPER_H = 100;
    public static final double WHITE_UPPER_S = 25;
    public static final double WHITE_UPPER_V = 300;

    // White color camera gain and exposure
    public static final double WHITE_CAMERA_GAIN = 40;  // Range: 0-255
    public static final double WHITE_CAMERA_EXPOSURE = 10;  // Range: 0-10000

    // Purple color HSV values
    public static final double PURPLE_LOWER_H = 100;
    public static final double PURPLE_LOWER_S = 10;
    public static final double PURPLE_LOWER_V = 140;

    public static final double PURPLE_UPPER_H = 250;
    public static final double PURPLE_UPPER_S = 80;
    public static final double PURPLE_UPPER_V = 270;

    // Purple color camera gain and exposure
    public static final double PURPLE_CAMERA_GAIN = 30;  // Range: 0-255
    public static final double PURPLE_CAMERA_EXPOSURE = 40;  // Range: 0-10000

    /**Green hex for logic tuning*/
    // Green color camera gain and exposure
    public static final double GREEN_CAMERA_GAIN_PAPER = 30;  // Range: 0-255
    public static final double GREEN_CAMERA_EXPOSURE_PAPER = 7;  // Range: 0-10000

    // White color HSV values
    public static final double GREEN_LOWER_H_PAPER = 60;
    public static final double GREEN_LOWER_S_PAPER = 60;
    public static final double GREEN_LOWER_V_PAPER = 64;

    public static final double GREEN_UPPER_H_PAPER = 75;
    public static final double GREEN_UPPER_S_PAPER = 220;
    public static final double GREEN_UPPER_V_PAPER = 270;

    /**Pink hex for logic tuning*/
    // Pink color camera gain and exposure
    public static final double PINK_CAMERA_GAIN_PAPER = 30;  // Range: 0-255
    public static final double PINK_CAMERA_EXPOSURE_PAPER = 7;  // Range: 0-10000

    // Pink color HSV values
    public static final double PINK_LOWER_H_PAPER = 120;
    public static final double PINK_LOWER_S_PAPER = 30;
    public static final double PINK_LOWER_V_PAPER = 54;

    public static final double PINK_UPPER_H_PAPER = 150;
    public static final double PINK_UPPER_S_PAPER = 300;
    public static final double PINK_UPPER_V_PAPER = 270;

    /**Yellow hex for logic tuning*/
    // yellow color camera gain and exposure
    public static final double YELLOW_CAMERA_GAIN_PAPER = 30;  // Range: 0-255
    public static final double YELLOW_CAMERA_EXPOSURE_PAPER = 7;  // Range: 0-10000

    // yellow color HSV values
    public static final double YELLOW_LOWER_H_PAPER = 89;
    public static final double YELLOW_LOWER_S_PAPER = 90;
    public static final double YELLOW_LOWER_V_PAPER = 64;

    public static final double YELLOW_UPPER_H_PAPER = 92;
    public static final double YELLOW_UPPER_S_PAPER = 220;
    public static final double YELLOW_UPPER_V_PAPER = 270;


    /**red prop hsv*/
    // Red color camera gain and exposure
    public static final double RED_CAMERA_GAIN_PROP = 30;  // Range: 0-255
    public static final double RED_CAMERA_EXPOSURE_PROP = 7;  // Range: 0-10000

    // yellow color HSV values
    public static final double RED_LOWER_H_PROP = 100;
    public static final double RED_LOWER_S_PROP = 90;
    public static final double RED_LOWER_V_PROP = 90;

    public static final double RED_UPPER_H_PROP = 180;
    public static final double RED_UPPER_S_PROP = 255;
    public static final double RED_UPPER_V_PROP = 255;

    /**red prop YCbCr*/

    // yellow color HSV values
    public static final double RED_LOWER_Y_PROP = 88;
    public static final double RED_LOWER_Cb_PROP = 80;
    public static final double RED_LOWER_Cr_PROP = 52;

    public static final double RED_UPPER_Y_PROP = 76;
    public static final double RED_UPPER_Cb_PROP = -42;
    public static final double RED_UPPER_Cr_PROP = 128;
}
