package org.firstinspires.ftc.teamcode.Auto.Methods;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.odometry;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

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

    default void collectStraight(ElapsedTime autoTimer, Drivetrain drive, Collection.intakeHeightState firstHeight, Collection.intakeHeightState secondHeight) throws InterruptedException {

        boolean reversingIntake = false;

        double intakeNormal = 5500;

        ElapsedTime reverseIntakeTimer = new ElapsedTime();

        Collection.intakePowerState previousState = Collection.intakePowerState.off;

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        collection.setIntakeHeight(firstHeight);
        collection.updateIntakeHeight();

        drive.setAllPower(0);

        collection.setState(Collection.intakePowerState.on);
        collection.updateIntakeState();

        boolean collectionDone = !sensors.LeftClawSensor.isPressed() && !sensors.RightClawSensor.isPressed();

        int counter = 0;

        while (autoTimer.milliseconds() < 26000 && !collectionDone){

            counter++;
            collectionDone = !sensors.LeftClawSensor.isPressed() && !sensors.RightClawSensor.isPressed();

            if (counter <= 6){

                sleep(40);

            } else if (counter > 6 && counter <= 12) {

                collection.setIntakeHeight(secondHeight);
                collection.updateIntakeHeight();

                sleep(40);

            } else if (counter == 13){

                drive.strafeLeft();

                sleep(200);

                drive.setAllPower(0);

            } else if (counter == 14){

                drive.strafeRight();
                sleep(200);
                drive.setAllPower(0);

            } else if (counter > 14 && counter <= 20) {

                sleep(40);

            } else if (counter > 20) {

                collectionDone = true;

            }

            if (collection.getIntakeCurrentUse() > intakeNormal && !reversingIntake){
                reversingIntake = true;
                reverseIntakeTimer.reset();
                previousState = collection.getPowerState();
                collection.setState(Collection.intakePowerState.reversed);
                collection.updateIntakeState();
            }

            if (reversingIntake && reverseIntakeTimer.milliseconds() > 100){
                collection.setState(previousState);
                collection.updateIntakeState();
                reversingIntake = false;
            }

        }

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        sleep(200);

    }

}
