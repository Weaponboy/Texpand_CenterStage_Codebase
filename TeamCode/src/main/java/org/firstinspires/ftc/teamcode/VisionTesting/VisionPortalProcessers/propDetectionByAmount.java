package org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
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

    boolean telemetryBool = false;

    public enum color{
        blue,
        red
    }

    public enum Side{
        right,
        left
    }

    private final propDetectionByAmount.color color;

    private final propDetectionByAmount.Side side;

    public propDetectionByAmount(Telemetry telemetry, Side startSide, color propColor){
        this.telemetry = telemetry;
        this.color = propColor;
        this.side = startSide;
        telemetryBool = true;
    }

    public propDetectionByAmount(Side startSide, color propColor){
        this.color = propColor;
        this.side = startSide;
        telemetryBool = false;
    }

    public Scalar MIN_THRESH_BLUE = new Scalar(16, 50, 50);
    public Scalar MAX_THRESH_BLUE = new Scalar(50, 255, 255);

    public Scalar MIN_THRESH_RED = new Scalar(110, 20, 50);
    public Scalar MAX_THRESH_RED = new Scalar(220, 255, 255);

    static final Rect rightOfScreen = new Rect(new Point(320, 0), new Point(640, 240));
    static final Rect leftOfScreen = new Rect(new Point(0, 0), new Point(320, 240));

    public double position1 = 0;
    public double position2 = 0;
    public double position3 = 0;

    int lastReadingLeft;
    int lastReadingRight;

    int RightPixels;
    int LeftPixels;

    int counter;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        counter++;

        displayMat = frame.clone();

        RightSide = frame.submat(rightOfScreen);
        LeftSide = frame.submat(leftOfScreen);

        Imgproc.cvtColor(RightSide, RightSide, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(LeftSide, LeftSide, Imgproc.COLOR_BGR2HSV);

        switch (color) {
            case blue:
                Core.inRange(RightSide, MIN_THRESH_BLUE, MAX_THRESH_BLUE, modifiedRight);
                Core.inRange(LeftSide, MIN_THRESH_BLUE, MAX_THRESH_BLUE, modifiedLeft);
                break;
            case red:
                Core.inRange(RightSide, MIN_THRESH_RED, MAX_THRESH_RED, modifiedRight);
                Core.inRange(LeftSide, MIN_THRESH_RED, MAX_THRESH_RED, modifiedLeft);
                break;
            default:
        }

        erode(modifiedRight, modifiedRight, new Mat(5, 5, CV_8U));
        erode(modifiedLeft, modifiedLeft, new Mat(5, 5, CV_8U));

        dilate(modifiedRight, modifiedRight, new Mat(5, 5, CV_8U));
        dilate(modifiedLeft, modifiedLeft, new Mat(5, 5, CV_8U));

//        if (counter > 10){
//            lastReadingLeft = LeftPixels;
//            lastReadingRight = RightPixels;
//            counter = 0;
//        }

        RightPixels = Core.countNonZero(modifiedRight);
        LeftPixels = Core.countNonZero(modifiedLeft);

//        if(counter == 0){
//            if(lastReadingRight-RightPixels > 7000){
//                position1 = 0;
//                position2 = 0;
//                position3 = 0;
//            } else if (lastReadingLeft-LeftPixels > 7000) {
//                position1 = 0;
//                position2 = 0;
//                position3 = 0;
//            }
//        }

        switch (side) {
            case right:
                if (RightPixels - LeftPixels > 1200){
                    position3++;
                }else if (LeftPixels - RightPixels > 1000){
                    position2++;
                }else{
                    position1++;
                }
                break;
            case left:
                if (RightPixels - LeftPixels > 4000){
                    position2++;
                }else if (LeftPixels - RightPixels > 4000){
                    position3++;
                }else{
                    position1++;
                }
                break;
            default:
        }

        if (position1 > position2 && position1 > position3){
            propPos = 1;
        }else if (position2 > position1 && position2 > position3){
            propPos = 2;
        }else{
            propPos = 3;
        }

        if (position1 > 50 && position2 > 50){
            position1 = 0;
            position2 = 0;
            position3 = 0;
        } else if (position2 > 50 && position3 > 50){
            position1 = 0;
            position2 = 0;
            position3 = 0;
        }else if (position1 > 50 && position3 > 50){
            position1 = 0;
            position2 = 0;
            position3 = 0;
        }

        RightSide.release();
        LeftSide.release();

        telemetry.addData("prop Pos", propPos);
        telemetry.addData("contoursRight.size()", RightPixels);
        telemetry.addData("contoursLeft.size()", LeftPixels);
        telemetry.addData("prop Pos 1", position1);
        telemetry.addData("prop Pos 2", position2);
        telemetry.addData("prop Pos 3", position3);
        telemetry.update();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}