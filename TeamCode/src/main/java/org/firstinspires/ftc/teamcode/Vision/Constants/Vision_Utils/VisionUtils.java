package org.firstinspires.ftc.teamcode.Vision.Constants.Vision_Utils;

import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;

public class VisionUtils {

    public static double IMG_WIDTH = 432;
    public static double IMG_HEIGHT = 240;
    public static final double X_FOV = 72;
    public static final double Y_FOV = 43;
    public static final double CAMERA_HEIGHT = 0.19;
    public static final double BACK_WEBCAM_HEIGHT = 0.19;
    public static Target curTarget = Target.BLUE_CONE;

    public static final double RING_HEIGHT = 20;
    public static final double TOWER_HEIGHT = 230;

    public static double SHOOTER_OFFSET_DISTANCE = 12;

    // Note: All measurements in CM

    public enum Target {
        RED_CONE, BLUE_CONE, BLUE_POWERSHOTS, RED_POWERSHOTS, OUT_OF_RANGE
    }

    public enum Auto {
        BLUE_OUTER, BLUE_INNER, RED_INNER, RED_OUTER
    }

    public enum PowerShot {
        PS_CLOSE, PS_MID, PS_FAR
    }

    public static enum RECT_OPTION {
        AREA, WIDTH, HEIGHT, X, Y
    }
    public static enum AXES {
        X, Y
    }

    public static double pixels2Degrees(double pixels, AXES axe) {
        return (axe == AXES.X) ? pixels * (X_FOV / IMG_WIDTH) : pixels * (Y_FOV / IMG_HEIGHT);
    }

    /**
     * Sorts n number of rectangles by RECT_OPTION in ascending order
     * @param n
     * @param option
     * @param rects
     * @return
     */
    public static List<Rect> sortRectsByMinOption(int n, RECT_OPTION option, List<Rect> rects){
        List<Rect> sorted_rects = new ArrayList<>();
        for (int j=0; j < n; j++){
            int beta_index = getMinIndex(rects, option);
            sorted_rects.add(rects.get(beta_index));

            rects.remove(beta_index);
            if (rects.size() == 0) break;
        }
        return sorted_rects;
    }

    /**
     * Sorts n number of rectangles by RECT_OPTION in descending order
     * @param n
     * @param option
     * @param rects
     * @return
     */
    public static List<Rect> sortRectsByMaxOption(int n, RECT_OPTION option, List<Rect> rects){
        List<Rect> sorted_rects = new ArrayList<>();
        for (int j=0; j < n; j++){
            int alpha_i = getMaxIndex(rects, option);
            sorted_rects.add(rects.get(alpha_i));

            rects.remove(alpha_i);
            if (rects.size() == 0) break;
        }
        return sorted_rects;
    }


    /**
     * Sorts contours by x-coordinate in ascending order
     * @param contours
     * @return
     */
    public static int findLeftMostContourIndex(List<MatOfPoint> contours){
        int index = 0;
        double minX = Integer.MAX_VALUE;
        for (int i=0; i < contours.size(); i++){
            MatOfPoint cnt = contours.get(i);
            double x = boundingRect(cnt).x;
            if (x < minX) {
                minX = x;
                index = i;
            }
        }
        return index;
    }

    /**
     * Sorts n number of contours by x-coordinate in ascending order
     * @param n
     * @param contours
     * @return
     */
    public static List<MatOfPoint> findNLeftMostContours(int n, List<MatOfPoint> contours){
        List<MatOfPoint> widest_contours = new ArrayList<>();
        for (int j=0; j < n; j++){
            int largest_index = findLeftMostContourIndex(contours);
            widest_contours.add(contours.get(largest_index));

            contours.remove(largest_index);
            if (contours.size() == 0) break;
        }

        for (MatOfPoint cnt : contours){
            cnt.release();
        }

        return widest_contours;
    }

    public static int findWidestContourIndex(List<MatOfPoint> contours){
        int index = 0;
        double maxWidth = 0;
        for (int i=0; i < contours.size(); i++){
            MatOfPoint cnt = contours.get(i);
            double width = boundingRect(cnt).width;
            if (width > maxWidth) {
                maxWidth = width;
                index = i;
            }
        }
        return index;
    }

    public static List<MatOfPoint> findNWidestContours(int n, List<MatOfPoint> contours){
        List<MatOfPoint> widest_contours = new ArrayList<>();
        for (int j=0; j < n; j++){
            int largest_index = findWidestContourIndex(contours);
            widest_contours.add(contours.get(largest_index));

            contours.remove(largest_index);
            if (contours.size() == 0) break;
        }

        for (MatOfPoint cnt : contours){
            cnt.release();
        }

        return widest_contours;
    }

    public static int findLargestContourIndex(List<MatOfPoint> contours) {
        int index = 0;
        double maxArea = 0;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint cnt = contours.get(i);
            double area = contourArea(cnt);
            if (area > maxArea) {
                maxArea = area;
                index = i;
            }
        }
        return index;
    }

    public static List<MatOfPoint> findNLargestContours(int n, List<MatOfPoint> contours) {
        List<MatOfPoint> new_contours = new ArrayList<>();

        for (int j = 0; j < n; j++) {
            int largest_index = findLargestContourIndex(contours);
            new_contours.add(contours.get(largest_index));

            contours.remove(largest_index);
            if (contours.size() == 0) break;
        }
        for (MatOfPoint cnt : contours){
            cnt.release();
        }
        return new_contours;
    }



    /**
     * Searches for the index of the item who's RECT_OPTION is the greatest
     * @param rects
     * @param option
     * @return
     */
    public static int getMaxIndex(List<Rect> rects, RECT_OPTION option){
        int alpha_index = 0;
        double max = Integer.MIN_VALUE;
        double cur = 0;
        for (int i=0; i < rects.size(); i++){

            switch (option){
                case X:
                    cur = rects.get(i).x;
                    break;

                case Y:
                    cur = rects.get(i).y;
                    break;

                case WIDTH:
                    cur = rects.get(i).width;
                    break;

                case HEIGHT:
                    cur = rects.get(i).height;
                    break;

                case AREA:
                    cur = rects.get(i).width * rects.get(i).height;
                    break;

            }
            if (cur > max) {
                max = cur;
                alpha_index = i;
            }
        }
        return alpha_index;
    }

    /**
     * Searches for the index of the item who's RECT_OPTION is the least
     * @param rects
     * @param option
     * @return
     */
    public static int getMinIndex(List<Rect> rects, RECT_OPTION option){
        int beta_index = 0;
        double min = Integer.MAX_VALUE;
        double cur = 0;
        for (int i=0; i < rects.size(); i++){

            switch (option){
                case X:
                    cur = rects.get(i).x;
                    break;

                case Y:
                    cur = rects.get(i).y;
                    break;

                case WIDTH:
                    cur = rects.get(i).width;
                    break;

                case HEIGHT:
                    cur = rects.get(i).height;
                    break;

                case AREA:
                    cur = rects.get(i).width * rects.get(i).height;
                    break;
            }
            if (cur < min) {
                min = cur;
                beta_index = i;
            }
        }
        return beta_index;
    }

}
