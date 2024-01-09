package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Sensors {

    HardwareMap hardwareMap;

    public VisionPortal portal;

    public WebcamName frontCam;

    public DistanceSensor RightClawSensor;
    public DistanceSensor LeftClawSensor;

    public AprilTagDetection rightTag;

    private AprilTagProcessor aprilTag;

    public void init(HardwareMap hmap){

        hardwareMap = hmap;

        frontCam = hardwareMap.get(WebcamName.class, "frontcam");

//        RightClawSensor = hardwareMap.get(DistanceSensor.class, "rightclaw");
//        LeftClawSensor = hardwareMap.get(DistanceSensor.class, "leftclaw");

    }

    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(frontCam);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);

        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        portal = builder.build();

    }

    public void getDetections() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            if (detection.id == 4 || detection.id == 5 || detection.id == 6){
                rightTag = detection;
            }

        }

    }

}
