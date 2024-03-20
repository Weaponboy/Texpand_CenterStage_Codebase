package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Methods;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.odometry;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;

public interface CycleMethods extends Auto_Methods {

    default void dropWhitePixels(Delivery.PixelsAuto pixelPlacement) throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.droppingWhites);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, pixelPlacement, odometry);

        while (!(delivery.getArmState() == Delivery.armState.readyToDrop)){
            delivery.updateArm(deliverySlides.getCurrentposition(), sensors.armSensor.isPressed(), pixelPlacement, odometry);
        }

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(200);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, pixelPlacement, odometry);

        deliverySlides.DeliverySlides(0, -1);

        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

    }

    default void dropWhitePixelsWait(Delivery.PixelsAuto pixelPlacement) throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.droppingWhites);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, pixelPlacement, odometry);


        while (!(delivery.getArmState() == Delivery.armState.readyToDrop)){
            delivery.updateArm(deliverySlides.getCurrentposition(), sensors.armSensor.isPressed(), pixelPlacement, odometry);
        }

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(200);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, pixelPlacement, odometry);

        deliverySlides.DeliverySlides(0, -1);

        while (deliverySlides.getCurrentposition() > 20){
            delivery.updateArm(deliverySlides.getCurrentposition(), false,  pixelPlacement, odometry);
        }

    }

}
