package org.firstinspires.ftc.teamcode.hardware.Method_Interfaces;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Sensors;

import java.util.Objects;

public interface Auto_Methods {

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Delivery delivery = new Delivery();

    Collection collection = new Collection();

    Sensors sensors = new Sensors();

    default void init(HardwareMap hardwareMap){
        delivery.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        collection.init(hardwareMap);

        sensors.init(hardwareMap);
    }

    default void dropYellowPixel() throws InterruptedException {

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        sleep(200);

        deliverySlides.DeliverySlides(400, 0.6);

        while (deliverySlides.getCurrentposition() < 390){}

        delivery.setArmTargetState(Delivery.armState.deliverAuto);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(1500);

        delivery.setGripperState(Delivery.targetGripperState.openRight);
        delivery.updateGrippers();

        sleep(1500);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(1000);

        deliverySlides.DeliverySlides(0, -0.6);

        sleep(500);

        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
        collection.updateIntakeHeight();

    }

    default void dropWhitePixels() throws InterruptedException {

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        sleep(200);

        deliverySlides.DeliverySlides(700, 0.6);

        while (deliverySlides.getCurrentposition() < 680){

            if (deliverySlides.getCurrentposition() > 150){
                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition());
            }

        }

        boolean armInPosition = false;

        while (!armInPosition){
            if (Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.deliverAuto) {
                armInPosition = true;
            }
        }

        sleep(200);

        delivery.setGripperState(Delivery.targetGripperState.openBoth);
        delivery.updateGrippers();

        sleep(500);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(100);

        deliverySlides.DeliverySlides(0, -0.6);

        while (deliverySlides.getCurrentposition() > 100){
            //wait
        }

        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
        collection.updateIntakeHeight();

    }

}
