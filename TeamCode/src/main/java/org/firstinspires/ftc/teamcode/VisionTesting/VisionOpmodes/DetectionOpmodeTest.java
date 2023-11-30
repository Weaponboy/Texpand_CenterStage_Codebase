package org.firstinspires.ftc.teamcode.VisionTesting.VisionOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.PropDetecterByHeight;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class DetectionOpmodeTest extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    
    PropDetecterByHeight propDetectorTest;
    private VisionPortal visionPortal;
    WebcamName webcamName;

   @Override
   public void init() {
       webcamName = hardwareMap.get(WebcamName.class, "frontCam");
//       greenPixel = new GreenPixelDetecter();
//       yellowPixel = new YellowPixelDetecter();
       propDetectorTest = new PropDetecterByHeight(PropDetecterByHeight.color.blue);
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
