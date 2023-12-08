package org.firstinspires.ftc.teamcode.VisionTesting.VisionOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.ColorRangePipeline;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.PropDetecterByHeight;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.RightTest;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class DetectionOpmodeTest extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ColorRangePipeline propDetectionByAmount = new ColorRangePipeline(telemetry);

    OpenCvCamera Texpandcamera;

   @Override
   public void init() {
       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

       WebcamName webcamName = hardwareMap.get(WebcamName.class, "frontCam");

       Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

       Texpandcamera.setPipeline(propDetectionByAmount);

       Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
           @Override
           public void onOpened() {
               Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
           }

           @Override
           public void onError(int errorCode) {


           }
       });

       telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
   }

    @Override
    public void init_loop() {
//        telemetry.addData("height", propDetectorTest.);
        telemetry.update();
   }

    @Override
    public void start() {
        Texpandcamera.stopStreaming();
   }

    @Override
    public void loop() {

    }
}
