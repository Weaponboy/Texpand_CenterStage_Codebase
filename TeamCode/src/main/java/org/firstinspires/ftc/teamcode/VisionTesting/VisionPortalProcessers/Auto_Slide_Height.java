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

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class Auto_Slide_Height implements VisionProcessor {

    Rect rect = new Rect();

    Mat modified = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    private Mat hierarchy = new Mat();

    private List<Rect> rects = new ArrayList<>();

    private List<Rect> OrderedByWidthrects = new ArrayList<>();

    private int FullRect = -1;

    private Rect TargetHighRect;

    public Scalar MIN_THRESH_BLACK = new Scalar(0, 0, 0);
    public Scalar MAX_THRESH_BLACK = new Scalar(180, 255, 80);

    Telemetry telemetry;

    public boolean stopSlides;

    public Auto_Slide_Height(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        frame.copyTo(modified);

        Imgproc.cvtColor(modified, modified, COLOR_RGB2HSV);

        inRange(modified, MIN_THRESH_BLACK, MAX_THRESH_BLACK, modified);

        erode(modified, modified, new Mat(5, 5, CV_8U));

        dilate(modified, modified, new Mat(5, 5, CV_8U));

        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        if (rects.size() > 0) {

            OrderedByWidthrects = VisionUtils.sortRectsByMaxOption(rects.size(), VisionUtils.RECT_OPTION.WIDTH, rects);

            //find the widths expected for a high pole and a medium pole
            for (int i = 0; i < OrderedByWidthrects.size(); i++) {
                if (OrderedByWidthrects.get(i).width > 620 && (OrderedByWidthrects.get(i).width < 700)) {
                    FullRect = i;
                }
            }

            if (FullRect > -1) {

                TargetHighRect = OrderedByWidthrects.get(FullRect);

                rect = new Rect(TargetHighRect.x, TargetHighRect.y, TargetHighRect.width, TargetHighRect.height);
                stopSlides = true;

            }else {
                stopSlides = false;

                rect = new Rect(5,5,5,5);
            }

        }else {
            rect = new Rect(0,0,0,0);
        }

        contours.clear();
        rects.clear();
        FullRect = -1;

        telemetry.addData("stop slides", stopSlides);
        telemetry.update();

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
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint purple = new Paint();
        purple.setColor(Color.MAGENTA);
        purple.setStyle(Paint.Style.STROKE);
        purple.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), purple);
    }

}
