package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Stack_2;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Preload.Auto_Methods;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Delivery_Slides;

public interface CycleMethods extends Auto_Methods {

    default void deployArm() throws InterruptedException {

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        deliverySlides.DeliverySlides(500, 1);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

    }

    default void dropPixels() throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

//        boolean armReady = false;

        sleep(500);

//        while (!armReady){
//
//            switch (delivery.getArmState()){
//                case delivery:
//                    armReady = true;
//                    break;
//                default:
//            }
//
//            delivery.updateArm(deliverySlides.getCurrentposition());
//
//        }

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(200);

    }

    default void retract() throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        deliverySlides.DeliverySlides(0, -0.5);

        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

    }

    default void retractWait() throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        deliverySlides.DeliverySlides(0, -0.5);

        while (deliverySlides.getCurrentposition() > 20){

        }

        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

    }

    default void collectPixels() throws InterruptedException {

        sleep(800);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        sleep(200);

        collection.setState(Collection.intakePowerState.reversed);
        collection.updateIntakeState();

        sleep(200);

        collection.setState(Collection.intakePowerState.off);
        collection.updateIntakeState();

    }

}
