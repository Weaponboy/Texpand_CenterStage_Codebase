package org.firstinspires.ftc.teamcode.hardware._;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Sensors {

    HardwareMap hardwareMap;

    public VisionPortal portal;

    public org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount;

    public WebcamName frontCam;

    public DistanceSensor backBoard;

    public TouchSensor RightClawSensor;
    public TouchSensor LeftClawSensor;

    public TouchSensor armSensor;

    public AprilTagDetection rightTag;

    private AprilTagProcessor aprilTag;

    public void init(HardwareMap hmap){

        hardwareMap = hmap;

        frontCam = hardwareMap.get(WebcamName.class, "frontcam");

        RightClawSensor = hardwareMap.get(TouchSensor.class, "rightclaw");
        LeftClawSensor = hardwareMap.get(TouchSensor.class, "leftclaw");

        armSensor = hardwareMap.get(TouchSensor.class, "armangle");

        backBoard = hardwareMap.get(DistanceSensor.class, "backboard");

    }

    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .build();

        portal = VisionPortal.easyCreateWithDefaults(frontCam, aprilTag);

    }

    public void initAprilTag(Telemetry telemetry, boolean red) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .build();

        if (red) {
            propDetectionByAmount  = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.color.red);
        } else {
            propDetectionByAmount  = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.color.blue);
        }

        portal = VisionPortal.easyCreateWithDefaults(frontCam, aprilTag, propDetectionByAmount);

    }

    public void getDetections() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.size() == 0){
            rightTag = null;
        }

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            if (detection.id == 4 || detection.id == 5 || detection.id == 6 || detection.id == 1 || detection.id == 2 || detection.id == 3){
                rightTag = detection;
            }else {
                rightTag = null;
            }

        }

    }

    public void getDetections(Telemetry telemetry) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.size() == 0){
            rightTag = null;
        }
        double arraysize = currentDetections.size();
        telemetry.addData("detection size", arraysize );
        telemetry.update();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

//            try {
//                telemetry.addData("detection size", currentDetections.size());
//                telemetry.update();
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//
//            telemetry.addData("detection size", currentDetections.size());
//            telemetry.update();

            if (detection.id == 4 || detection.id == 5 || detection.id == 6 || detection.id == 1 || detection.id == 2 || detection.id == 3){
                rightTag = detection;
            }else {
                rightTag = null;
            }

        }

    }


}
