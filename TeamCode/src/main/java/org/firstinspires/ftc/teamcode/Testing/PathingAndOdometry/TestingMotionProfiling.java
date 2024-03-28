package org.firstinspires.ftc.teamcode.Testing.PathingAndOdometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;

@TeleOp
public class TestingMotionProfiling extends LinearOpMode {

    Delivery_Slides deliverySlides = new Delivery_Slides();

    @Override
    public void runOpMode() throws InterruptedException {

        deliverySlides.init(hardwareMap);
        deliverySlides.setTargetPos(200);
        deliverySlides.motionProfile();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                deliverySlides.setTargetPos(200);
                deliverySlides.motionProfile();
            }

            if (gamepad1.b){
                deliverySlides.setTargetPos(800);
                deliverySlides.motionProfile();
            }

            deliverySlides.SlidesBothPower(deliverySlides.getPower());

            telemetry.addData("position", deliverySlides.getCurrentposition());
            telemetry.addData("power", deliverySlides.getPower());
            telemetry.update();

        }

    }

}
