package org.firstinspires.ftc.teamcode.hardware.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class Sensors {

    HardwareMap hardwareMap;

    public VisionPortal portal;

    public WebcamName frontCam;

    public void init(HardwareMap hmap){
        hardwareMap = hmap;

        frontCam = hardwareMap.get(WebcamName.class, "frontCam");
    }
}
