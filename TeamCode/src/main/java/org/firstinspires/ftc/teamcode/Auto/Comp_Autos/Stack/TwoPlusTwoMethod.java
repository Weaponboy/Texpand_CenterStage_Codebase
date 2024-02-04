package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Stack;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Preload.Auto_Methods;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;

public interface TwoPlusTwoMethod extends Auto_Methods {

    default void deployArm() throws InterruptedException {

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        deliverySlides.DeliverySlides(700, 0.6);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        while (deliverySlides.getCurrentposition() < 200){

            if (deliverySlides.getCurrentposition() > 150){
                delivery.setArmTargetState(Delivery.armState.delivery);
                delivery.updateArm(deliverySlides.getCurrentposition());
            }

        }

    }

    default void dropPixels() throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        boolean armReady = false;

        while (!armReady){

            switch (delivery.getArmState()){
                case delivery:
                    armReady = true;
                    break;
                default:
            }

            delivery.updateArm(deliverySlides.getCurrentposition());

        }

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

    default void collectPixels(HardwareMap hardwareMap) throws InterruptedException {

        sensors.init(hardwareMap);

        collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
        collection.updateIntakeHeight();

        boolean abortAndTry = false;
        boolean gotBoth = false;

        while (!gotBoth && !abortAndTry){

            gotBoth = sensors.LeftClawSensor.isPressed() && sensors.RightClawSensor.isPressed();

            if (gotBoth){
                delivery.setRightGripperState(Delivery.rightGripperState.closed);
            }

            if (autoTimer.milliseconds() > 26000){
                abortAndTry = true;
            }

        }

        collection.setState(Collection.intakePowerState.reversed);
        collection.updateIntakeState();

        sleep(200);

        collection.setState(Collection.intakePowerState.off);
        collection.updateIntakeState();

    }

}