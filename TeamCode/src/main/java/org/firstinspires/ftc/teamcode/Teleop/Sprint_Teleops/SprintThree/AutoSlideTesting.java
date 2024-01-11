package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintThree;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.Auto_Slide_Height;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Disabled
public class AutoSlideTesting extends LinearOpMode {

    Delivery_Slides deliverySlides = new Delivery_Slides();

    WebcamName backCam;
    VisionPortal slideHeight;
    Auto_Slide_Height autoSlideHeight = new Auto_Slide_Height(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {

        deliverySlides.init(hardwareMap);
        backCam = hardwareMap.get(WebcamName.class, "frontcam");
        slideHeight = VisionPortal.easyCreateWithDefaults(backCam, autoSlideHeight);

        waitForStart();

        while (deliverySlides.getCurrentposition() < 1100){
            deliverySlides.SlidesBothPower(1);
        }

        boolean SlideSafetyHeight = deliverySlides.getCurrentposition() > 2000;
        boolean SlideSafetyBottom = deliverySlides.getCurrentposition() < 5;

        while (!autoSlideHeight.stopSlides && !SlideSafetyHeight){
            SlideSafetyHeight = deliverySlides.getCurrentposition() > 2000;

            deliverySlides.SlidesBothPower(1);

        }

        deliverySlides.SlidesBothPower(0.0008);

        while (opModeIsActive()){
            telemetry.addData("de height", deliverySlides.getCurrentposition());
            telemetry.update();
        }

    }
}
