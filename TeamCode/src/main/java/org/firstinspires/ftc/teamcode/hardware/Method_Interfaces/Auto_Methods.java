package org.firstinspires.ftc.teamcode.hardware.Method_Interfaces;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;
import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Sensors;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
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

//        sensors.init(hardwareMap);
    }

    default void dropYellowPixel() throws InterruptedException {

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
                delivery.setArmTargetState(Delivery.armState.delivery);
                delivery.updateArm(deliverySlides.getCurrentposition());
            }

        }

        boolean armInPosition = false;

        while (!armInPosition){

            switch (delivery.getArmState()){
                case delivery:
                    armInPosition = true;
                    break;
                default:
            }

            delivery.updateArm(deliverySlides.getCurrentposition());
        }

        sleep(200);

        delivery.setGripperState(Delivery.GripperState.closed);
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
