package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class Sensors {

    HardwareMap hardwareMap;

    public VisionPortal portal;

    public WebcamName frontCam;

    public DistanceSensor RightClawSensor;
    public DistanceSensor LeftClawSensor;


    public void init(HardwareMap hmap){

        hardwareMap = hmap;

        frontCam = hardwareMap.get(WebcamName.class, "frontcam");

        RightClawSensor = hardwareMap.get(DistanceSensor.class, "rightclaw");
        LeftClawSensor = hardwareMap.get(DistanceSensor.class, "leftclaw");

    }
}
