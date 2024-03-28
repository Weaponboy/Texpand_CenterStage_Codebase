package org.firstinspires.ftc.teamcode.Vision.VisionOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.pixelDetection;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Disabled
public class PixelDetection extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    
    pixelDetection propDetectorTest;
    private VisionPortal visionPortal;
    WebcamName webcamName;

   @Override
   public void init() {
       webcamName = hardwareMap.get(WebcamName.class, "frontCam");
       propDetectorTest = new pixelDetection();
       visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, propDetectorTest);

       telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
   }

    @Override
    public void init_loop() {
//        telemetry.addData("height", propDetectorTest.);
        telemetry.update();
   }

    @Override
    public void start() {
        visionPortal.stopStreaming();
   }

    @Override
    public void loop() {

    }
}
