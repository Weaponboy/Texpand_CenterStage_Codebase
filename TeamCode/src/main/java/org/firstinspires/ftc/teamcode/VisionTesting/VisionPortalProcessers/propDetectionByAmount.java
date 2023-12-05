package org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class propDetectionByAmount implements VisionProcessor {

    Mat RightSide = new Mat();
    Mat modifiedRight = new Mat();

    Mat LeftSide = new Mat();
    Mat modifiedLeft = new Mat();

    Mat displayMat = new Mat();

    Telemetry telemetry;

    public enum color{
        blue,
        red
    }

    private final propDetectionByAmount.color color;

    public propDetectionByAmount(Telemetry telemetry, boolean debugging, color propColor){
        this.telemetry = telemetry;
        this.color = propColor;
    }

    public Scalar MIN_THRESH_BLUE = new Scalar(5, 90, 110);
    public Scalar MAX_THRESH_BLUE = new Scalar(40, 255, 255);

    public Scalar MIN_THRESH_RED = new Scalar(140, 50, 50);
    public Scalar MAX_THRESH_RED = new Scalar(220, 255, 255);

    static final Rect rightOfScreen = new Rect(new Point(320, 0), new Point(640, 480));
    static final Rect leftOfScreen = new Rect(new Point(0, 0), new Point(320, 480));

    int propPos;

    public double position1 = 0;
    public double position2 = 0;
    public double position3 = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        displayMat = frame.clone();

        RightSide = frame.submat(rightOfScreen);
        LeftSide = frame.submat(leftOfScreen);

        Imgproc.cvtColor(RightSide, RightSide, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(LeftSide, LeftSide, Imgproc.COLOR_BGR2HSV);

        switch (color) {
            case blue:
                Core.inRange(RightSide, MIN_THRESH_RED, MAX_THRESH_RED, modifiedRight);
                Core.inRange(LeftSide, MIN_THRESH_RED, MAX_THRESH_RED, modifiedLeft);
                break;
            case red:
                Core.inRange(RightSide, MIN_THRESH_BLUE, MAX_THRESH_BLUE, modifiedRight);
                Core.inRange(LeftSide, MIN_THRESH_BLUE, MAX_THRESH_BLUE, modifiedLeft);
                break;
            default:
        }

        erode(modifiedRight, modifiedRight, new Mat(5, 5, CV_8U));
        erode(modifiedLeft, modifiedLeft, new Mat(5, 5, CV_8U));

        dilate(modifiedRight, modifiedRight, new Mat(5, 5, CV_8U));
        dilate(modifiedLeft, modifiedLeft, new Mat(5, 5, CV_8U));

        int RightPixels = Core.countNonZero(modifiedRight);
        int LeftPixels = Core.countNonZero(modifiedLeft);

        if (RightPixels - LeftPixels > 200){
            position2++;
        }else if (LeftPixels - RightPixels > 200){
            position1++;
        }else{
            position3++;
        }

        if (position1 > position2 && position1 > position3){
            propPos = 1;
        }else if (position2 > position1 && position2 > position3){
            propPos = 2;
        }else{
            propPos = 3;
        }

        telemetry.addData("prop Pos 1", position1);
        telemetry.addData("prop Pos 2", position2);
        telemetry.addData("prop Pos 3", position3);
        telemetry.addData("modifyefLeft", modifiedLeft.width());
        telemetry.addData("prop Pos", propPos);
        telemetry.addData("contoursRight.size()", RightPixels);
        telemetry.addData("contoursLeft.size()", LeftPixels);
        telemetry.update();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}