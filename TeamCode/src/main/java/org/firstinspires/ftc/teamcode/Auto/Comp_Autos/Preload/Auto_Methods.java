package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Preload;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware._.Sensors;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public interface Auto_Methods {

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Delivery delivery = new Delivery();

    Collection collection = new Collection();

    Sensors sensors = new Sensors();

    ElapsedTime autoTimer = new ElapsedTime();

    default void init(HardwareMap hardwareMap){
        delivery.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        collection.init(hardwareMap);
    }

    default void dropYellowPixel() throws InterruptedException {

        deliverySlides.DeliverySlides(220, 1);

        while (deliverySlides.getCurrentposition() < 210){}

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

        boolean reachedTarget = false;

        while (!reachedTarget){
            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
            delivery.updateArm(deliverySlides.getCurrentposition());
        }

        delivery.setRightGripperState(Delivery.rightGripperState.openDeliver);
        delivery.updateGrippers();

        sleep(400);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(500);

        deliverySlides.DeliverySlides(0, -0.6);

    }

    default void dropYellowPixel(Telemetry telemetry) throws InterruptedException{

        deliverySlides.DeliverySlides(220, 1);

        while (deliverySlides.getCurrentposition() < 210){}

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

        boolean reachedTarget = false;

        while (!reachedTarget){
            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
            delivery.updateArm(deliverySlides.getCurrentposition());
            telemetry.addData("state", delivery.getArmState());
            telemetry.update();
        }

        delivery.setRightGripperState(Delivery.rightGripperState.openDeliver);
        delivery.updateGrippers();

        sleep(400);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        deliverySlides.DeliverySlides(0, -0.6);

    }

    default void dropYellowPixelDrive() throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(800);

        delivery.setRightGripperState(Delivery.rightGripperState.openDeliver);
        delivery.updateGrippers();

        deliverySlides.DeliverySlides(320, 0.6);

        while (deliverySlides.getCurrentposition() < 310){}

        sleep(400);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(1000);

        deliverySlides.DeliverySlides(0, -0.6);

    }

    default void dropYellowPixelWait() throws InterruptedException {

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        sleep(200);

        deliverySlides.DeliverySlides(220, 0.6);

        while (deliverySlides.getCurrentposition() < 210){}

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(1000);

        delivery.setRightGripperState(Delivery.rightGripperState.openDeliver);
        delivery.updateGrippers();

        deliverySlides.DeliverySlides(320, 0.6);

        while (deliverySlides.getCurrentposition() < 310){}

        sleep(400);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(1000);

        deliverySlides.DeliverySlides(0, -0.6);

        while (deliverySlides.getCurrentposition() > 30){

        }

    }


    default Vector2D getDetections(AprilTagProcessor aprilTag) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            if (detection.id == 4 || detection.id == 5 || detection.id == 6){

                double NewY;
                double NewX;

                double aprilTagOffset;

                Vector2D newPosition;

                if (sensors.rightTag.id == 4){
                    aprilTagOffset = getRealCoords(75);
                }else if (sensors.rightTag.id == 5){
                    aprilTagOffset = getRealCoords(90);
                }else{
                    aprilTagOffset = getRealCoords(105);
                }

                double realNewY = -(Math.cos(sensors.rightTag.ftcPose.bearing)) * (sensors.rightTag.ftcPose.x * 0.1);
                double realNewX = (sensors.rightTag.ftcPose.y * 0.1);

                NewY = aprilTagOffset + realNewY;
                NewX = 360 - (realNewX + 45);

                newPosition = new Vector2D(NewX, NewY - 12.5);

                return newPosition;

            }

        }

        return null;
    }

}
