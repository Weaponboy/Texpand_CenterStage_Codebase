package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Methods;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;

public interface CycleMethods extends Auto_Methods {

    default void deployArm() throws InterruptedException {

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        deliverySlides.DeliverySlides(600, 1);

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

        boolean reachedTarget = false;

        while (!reachedTarget){
            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
            delivery.updateArm(deliverySlides.getCurrentposition());
        }

        sleep(200);

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(200);

    }

    default void retract() throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        deliverySlides.DeliverySlides(0, -1);

        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

    }

    default void retractWait() throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        deliverySlides.DeliverySlides(0, -1);

        while (deliverySlides.getCurrentposition() > 20){

        }

        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

    }

    default void dropWhitePixels(double armPos, Odometry odometry, Telemetry telemetry) throws InterruptedException {

        sensors.portal.setProcessorEnabled(sensors.propDetectionByAmount, false);

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, true);

        boolean reachedTarget = false;

        double timeToWaitSideMove = (Math.abs(delivery.RotateArm.getPosition() - delivery.ArmPositionMid) * 180) * 5;
        waitForSideArm.reset();

        boolean waitDone = false;

        while (!waitDone){

            if (delivery.RotateArm.getPosition() < armPos){
                delivery.RotateArm.setPosition(delivery.RotateArm.getPosition() + 0.006);
            } else if (delivery.RotateArm.getPosition() > armPos) {
                delivery.RotateArm.setPosition(delivery.RotateArm.getPosition() - 0.006);
            }

            delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

            if (delivery.RotateArm.getPosition() > (armPos-0.01) && delivery.RotateArm.getPosition() < (armPos+0.01)){
                waitDone = true;
            }

        }

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

        reachedTarget = false;

        while (!reachedTarget){

            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
            delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

        }

        sleep(1000);

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(400);

        delivery.RotateArm.setPosition(delivery.ArmPositionMid);

        delivery.ArmExtension.setPosition(1);

        delivery.setArmTargetState(Delivery.armState.collect);

        boolean reachedTargetCollection = false;

        while (!reachedTargetCollection){

            reachedTargetCollection = delivery.getArmState() == Delivery.armState.collect;
            delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

            sensors.getDetections();
            resetOdo(odometry, telemetry);

        }

        deliverySlides.DeliverySlides(0, -1);

    }

    default void dropWhitePixelsWait(double armPos, Odometry odometry, Telemetry telemetry) throws InterruptedException {

        sensors.portal.setProcessorEnabled(sensors.propDetectionByAmount, false);

        boolean reachedTarget = false;

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

        while (!reachedTarget){
            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
            delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);
        }

        odometry.update();

        double heading = odometry.heading;

        if (odometry.heading > 180 && odometry.heading < 270) {
            heading = odometry.heading - 180;
        } else if (odometry.heading > 90 && odometry.heading < 180) {
            heading = odometry.heading - 180;
        } else if (odometry.heading > 270) {
            heading = 90;
        } else if (odometry.heading < 90) {
            heading = -90;
        }

        delivery.setSecondRotate(0.5 + (heading / 180));

        boolean backboardFalse = false;

        while (!backboardFalse){

            delivery.ArmExtension.setPosition(Range.clip((delivery.ArmExtension.getPosition() - 0.006), 0.4, 1));

            if (sensors.armSensor.isPressed()){
                backboardFalse = true;
            }

        }

        sleep(100);

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(400);

        delivery.RotateArm.setPosition(delivery.ArmPositionMid);

        delivery.ArmExtension.setPosition(1);

        delivery.setArmTargetState(Delivery.armState.collect);

        boolean reachedTargetCollection = false;

        while (!reachedTargetCollection){

            reachedTargetCollection = delivery.getArmState() == Delivery.armState.collect;
            delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

            sensors.getDetections();
            resetOdo(odometry, telemetry);

        }

        deliverySlides.DeliverySlides(0, -1);

        while (deliverySlides.getCurrentposition() > 20){

        }

    }


}
