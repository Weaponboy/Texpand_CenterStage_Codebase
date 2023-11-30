package org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.VisionTesting.Constants.Vision_Utils.VisionUtils;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class pixelDetection implements VisionProcessor {

    public Rect rect = new Rect(20, 20, 50, 50);

    public Mat greenPixel = new Mat();
    public Mat yellowPixel = new Mat();
    public Mat purplePixel = new Mat();
    public Mat whitePixel = new Mat();

    public Scalar MIN_THRESH_PURPLE = new Scalar(84, 82, 87);
    public Scalar MAX_THRESH_PURPLE = new Scalar(156, 155, 155);

    public Scalar MIN_THRESH_GREEN = new Scalar(22, 74, 21);
    public Scalar MAX_THRESH_GREEN = new Scalar(80, 120, 56);

    public Scalar MIN_THRESH_WHITE = new Scalar(255, 255, 255);
    public Scalar MAX_THRESH_WHITE = new Scalar(255, 255, 255 );

    public Scalar MIN_THRESH_YELLOW = new Scalar(117, 108, 57);
    public Scalar MAX_THRESH_YELLOW = new Scalar(157, 153, 105);

    public enum pixelColor{
        purple,
        yellow,
        green,
        white,
        all
    }

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private ArrayList<MatOfPoint> yellowcontours = new ArrayList<>();
    private ArrayList<MatOfPoint> whitecontours = new ArrayList<>();
    private ArrayList<MatOfPoint> purplecontours = new ArrayList<>();
    private ArrayList<MatOfPoint> greencontours = new ArrayList<>();

    private Mat hierarchy = new Mat();

    private List<Rect> whiterects = new ArrayList<>();
    private List<Rect> yellowrects = new ArrayList<>();
    private List<Rect> purplerects = new ArrayList<>();
    private List<Rect> greenrects = new ArrayList<>();

    private List<Rect> GreenPixelsRawHeight = new ArrayList<>();
    private List<Rect> GreenPixelsRawWidth = new ArrayList<>();
    private List<Rect> GreenPixels = new ArrayList<>();

    private List<Rect> PurplePixelsRawHeight = new ArrayList<>();
    private List<Rect> PurplePixelsRawWidth = new ArrayList<>();
    private List<Rect> PurplePixels = new ArrayList<>();

    private List<Rect> WhitePixelsRawHeight = new ArrayList<>();
    private List<Rect> WhitePixelsRawWidth = new ArrayList<>();
    private List<Rect> WhitePixels = new ArrayList<>();

    private List<Rect> YellowPixelsRawHeight = new ArrayList<>();
    private List<Rect> YellowPixelsRawWidth = new ArrayList<>();
    private List<Rect> YellowPixels = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        frame.copyTo(greenPixel);
        frame.copyTo(whitePixel);
        frame.copyTo(yellowPixel);
        frame.copyTo(purplePixel);

        Imgproc.cvtColor(greenPixel, greenPixel, COLOR_RGB2HSV);
        Imgproc.cvtColor(yellowPixel, yellowPixel, COLOR_RGB2HSV);
        Imgproc.cvtColor(purplePixel, purplePixel, COLOR_RGB2HSV);
        Imgproc.cvtColor(whitePixel, whitePixel, COLOR_RGB2HSV);

        inRange(greenPixel, MIN_THRESH_GREEN, MAX_THRESH_GREEN, greenPixel);
        inRange(yellowPixel, MIN_THRESH_YELLOW, MAX_THRESH_YELLOW, yellowPixel);
        inRange(purplePixel, MIN_THRESH_PURPLE, MAX_THRESH_PURPLE, purplePixel);
        inRange(whitePixel, MIN_THRESH_WHITE, MAX_THRESH_WHITE, whitePixel);

        //erode image
        erode(whitePixel, whitePixel, new Mat(5, 5, CV_8U));
        erode(purplePixel, purplePixel, new Mat(5, 5, CV_8U));
        erode(yellowPixel, yellowPixel, new Mat(5, 5, CV_8U));
        erode(greenPixel, greenPixel, new Mat(5, 5, CV_8U));

        //dilate image
        dilate(whitePixel, whitePixel, new Mat(5, 5, CV_8U));
        dilate(purplePixel, purplePixel, new Mat(5, 5, CV_8U));
        dilate(yellowPixel, yellowPixel, new Mat(5, 5, CV_8U));
        dilate(greenPixel, greenPixel, new Mat(5, 5, CV_8U));

        //find outlines of the objects of the colours in the range
        findContours(whitePixel, whitecontours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(purplePixel, purplecontours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(yellowPixel, yellowcontours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(greenPixel, greencontours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < whitecontours.size(); i++){
            Rect rect = boundingRect(whitecontours.get(i));
            whiterects.add(rect);
        }

        for (int i = 0; i < yellowcontours.size(); i++){
            Rect rect = boundingRect(yellowcontours.get(i));
            yellowrects.add(rect);
        }

        for (int i = 0; i < purplecontours.size(); i++){
            Rect rect = boundingRect(purplecontours.get(i));
            purplerects.add(rect);
        }

        for (int i = 0; i < greencontours.size(); i++){
            Rect rect = boundingRect(greencontours.get(i));
            greenrects.add(rect);
        }

        if (greenrects.size() > 0) {

            GreenPixelsRawHeight = VisionUtils.sortRectsByMaxOption(greenrects.size(), VisionUtils.RECT_OPTION.HEIGHT, greenrects);
            GreenPixelsRawWidth = VisionUtils.sortRectsByMaxOption(greenrects.size(), VisionUtils.RECT_OPTION.WIDTH, greenrects);

            for (int i = 0; i < GreenPixelsRawHeight.size(); i++) {
                if (Math.abs((GreenPixelsRawHeight.get(i).height) * (GreenPixelsRawWidth.get(i).width)) > 0.75 && Math.abs((GreenPixelsRawHeight.get(i).height) * (GreenPixelsRawWidth.get(i).width)) < 0.85) {
                    Rect rect = GreenPixelsRawHeight.get(i);
                    GreenPixels.set(0,rect);
                } else {
                    Rect rect = new Rect(0, 0, 0, 0);
                    GreenPixels.add(rect);
                }
            }
        }
//
//        if (yellowrects.size() > 0) {
//            YellowPixelsRawHeight = VisionUtils.sortRectsByMaxOption(yellowrects.size(), VisionUtils.RECT_OPTION.HEIGHT, yellowrects);
//            YellowPixelsRawWidth = VisionUtils.sortRectsByMaxOption(yellowrects.size(), VisionUtils.RECT_OPTION.WIDTH, yellowrects);
//
//            for (int i = 0; i < YellowPixelsRawHeight.size(); i++) {
//                if (Math.abs((YellowPixelsRawHeight.get(i).height) * (YellowPixelsRawWidth.get(i).width)) > 0.75 && Math.abs((YellowPixelsRawHeight.get(i).height) * (YellowPixelsRawWidth.get(i).width)) < 0.85) {
//                    YellowPixels.clear();
//                    Rect rect = YellowPixelsRawHeight.get(i);
//                    YellowPixels.add(rect);
//                } else {
//                    Rect rect = new Rect(0, 0, 0, 0);
//                    YellowPixels.add(rect);
//                }
//            }
//        }
//
//        if (whiterects.size() > 0) {
//            WhitePixelsRawHeight = VisionUtils.sortRectsByMaxOption(whiterects.size(), VisionUtils.RECT_OPTION.HEIGHT, whiterects);
//            WhitePixelsRawWidth = VisionUtils.sortRectsByMaxOption(whiterects.size(), VisionUtils.RECT_OPTION.WIDTH, whiterects);
//
//            for (int i = 0; i < WhitePixelsRawHeight.size(); i++) {
//                if (Math.abs((WhitePixelsRawHeight.get(i).height) * (WhitePixelsRawWidth.get(i).width)) > 0.75 && Math.abs((WhitePixelsRawHeight.get(i).height) * (WhitePixelsRawWidth.get(i).width)) < 0.85) {
//                    WhitePixels.clear();
//                    Rect rect = WhitePixelsRawHeight.get(i);
//                    WhitePixels.add(rect);
//                } else {
//                    Rect rect = new Rect(0, 0, 0, 0);
//                    WhitePixels.add(rect);
//                }
//            }
//        }
//
//        if (purplerects.size() > 0) {
//
//            PurplePixelsRawHeight = VisionUtils.sortRectsByMaxOption(purplerects.size(), VisionUtils.RECT_OPTION.HEIGHT, purplerects);
//            PurplePixelsRawWidth = VisionUtils.sortRectsByMaxOption(purplerects.size(), VisionUtils.RECT_OPTION.WIDTH, purplerects);
//
//            for (int i = 0; i < PurplePixelsRawHeight.size(); i++) {
//                if (Math.abs((PurplePixelsRawHeight.get(i).height) * (PurplePixelsRawWidth.get(i).width)) > 0.75 && Math.abs((PurplePixelsRawHeight.get(i).height) * (PurplePixelsRawWidth.get(i).width)) < 0.85) {
//                    PurplePixels.clear();
//                    Rect rect = PurplePixelsRawHeight.get(i);
//                    PurplePixels.add(rect);
//                } else {
//                    Rect rect = new Rect(0, 0, 0, 0);
//                    PurplePixels.add(rect);
//                }
//            }
//        }

        whitecontours.clear();
        yellowcontours.clear();
        greencontours.clear();
        purplecontours.clear();

        whiterects.clear();
        yellowrects.clear();
        greenrects.clear();
        purplerects.clear();

        return null;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {

        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);

        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint purple = new Paint();
        purple.setColor(Color.MAGENTA);
        purple.setStyle(Paint.Style.STROKE);
        purple.setStrokeWidth(scaleCanvasDensity * 4);

        Paint green = new Paint();
        green.setColor(Color.GREEN);
        green.setStyle(Paint.Style.STROKE);
        green.setStrokeWidth(scaleCanvasDensity * 4);

        Paint yellow = new Paint();
        yellow.setColor(Color.YELLOW);
        yellow.setStyle(Paint.Style.STROKE);
        yellow.setStrokeWidth(scaleCanvasDensity * 4);

        Paint white = new Paint();
        white.setColor(Color.WHITE);
        white.setStyle(Paint.Style.STROKE);
        white.setStrokeWidth(scaleCanvasDensity * 4);

        if (PurplePixels.size() > 0){
            canvas.drawRect(makeGraphicsRect(PurplePixels.get(0), scaleBmpPxToCanvasPx), purple);
        }else if(GreenPixels.size() > 0){
            canvas.drawRect(makeGraphicsRect(GreenPixels.get(0), scaleBmpPxToCanvasPx), green);
        }else if(WhitePixels.size() > 0){
            canvas.drawRect(makeGraphicsRect(WhitePixels.get(0), scaleBmpPxToCanvasPx), white);
        }else if(YellowPixels.size() > 0){
            canvas.drawRect(makeGraphicsRect(YellowPixels.get(0), scaleBmpPxToCanvasPx), yellow);
        }

    }
}

