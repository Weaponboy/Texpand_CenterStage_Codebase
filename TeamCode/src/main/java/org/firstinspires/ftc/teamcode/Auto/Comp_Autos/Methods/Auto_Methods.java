package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Methods;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;
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

    ElapsedTime waitForSideArm = new ElapsedTime();

    default void init(HardwareMap hardwareMap){

        delivery.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        collection.init(hardwareMap);

        sensors.init(hardwareMap);
    }

    default void dropYellowPixel() throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

        boolean reachedTarget = false;

        while (!reachedTarget){

            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
            delivery.updateArm(deliverySlides.getCurrentposition());

        }

        delivery.setRightGripperState(Delivery.rightGripperState.openDeliver);
        delivery.updateGrippers();

        sleep(200);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        deliverySlides.DeliverySlides(0, -1);

    }

    default void dropYellowPixel(double armPos, Odometry odometry, Telemetry telemetry) throws InterruptedException {

        sensors.portal.setProcessorEnabled(sensors.propDetectionByAmount, false);

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, true);

        boolean reachedTarget = false;

//        while (!reachedTarget){
//
//            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
//            delivery.updateArm(deliverySlides.getCurrentposition(), odometry, true);
//
//            sensors.getDetections();
//            resetOdo(odometry, telemetry);
//
//        }

        double timeToWaitSideMove = (Math.abs(delivery.RotateArm.getPosition() - delivery.ArmPositionMid) * 180) * 8;
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

            sensors.getDetections();
            resetOdo(odometry, telemetry);

        }

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

        reachedTarget = false;

        while (!reachedTarget){

            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
            delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

            sensors.getDetections();
            resetOdo(odometry, telemetry);

        }

        delivery.setLeftGripperState(Delivery.leftGripperState.openDeliver);
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

    default void dropYellowPixel(boolean both) throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

        boolean reachedTarget = false;

        while (!reachedTarget){
            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
            delivery.updateArm(deliverySlides.getCurrentposition());
        }

        delivery.setGripperState(Delivery.GripperState.openDeliver);
        delivery.updateGrippers();

        sleep(200);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        deliverySlides.DeliverySlides(0, -1);

    }

    default void dropYellowPixelWait(double armPos) throws InterruptedException {

        deliverySlides.DeliverySlides(350, 1);

        while (deliverySlides.getCurrentposition() < 345){}

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition());

        boolean reachedTarget = false;

        while (!reachedTarget){
            reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
            delivery.updateArm(deliverySlides.getCurrentposition());
        }

        delivery.RotateArm.setPosition(armPos);

        sleep((long) ((Math.abs(delivery.RotateArm.getPosition() - armPos) * 180) * 15));

        delivery.setRightGripperState(Delivery.rightGripperState.openDeliver);
        delivery.updateGrippers();

        sleep(400);

        delivery.RotateArm.setPosition(delivery.ArmPositionMid);
        delivery.setArmTargetState(Delivery.armState.collect);

        sleep((long) ((Math.abs(delivery.RotateArm.getPosition() - delivery.ArmPositionMid) * 180) * 5));

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(400);

        deliverySlides.DeliverySlides(0, -1);

        while (deliverySlides.getCurrentposition() > 30){
        }

    }

    default void dropYellowPixelWait() throws InterruptedException {

        deliverySlides.DeliverySlides(350, 1);

        while (deliverySlides.getCurrentposition() < 345){}

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

        sleep(400);

        deliverySlides.DeliverySlides(0, -1);

        while (deliverySlides.getCurrentposition() > 30){
        }

    }

    default void resetOdo(Odometry odometry, Telemetry telemetry){

        if (!(sensors.rightTag == null)){

            if (sensors.rightTag.id == 1 || sensors.rightTag.id == 2 || sensors.rightTag.id == 3){

                double NewY;
                double NewX;

                double aprilTagOffset;

                Vector2D newPosition;

                if (sensors.rightTag.id == 1){
                    aprilTagOffset = getRealCoords(75);
                }else if (sensors.rightTag.id == 2){
                    aprilTagOffset = getRealCoords(90);
                }else{
                    aprilTagOffset = getRealCoords(105);
                }

                double realNewX = (sensors.rightTag.ftcPose.y * 0.1);
                double realNewY = (sensors.rightTag.ftcPose.x * 0.1);

                NewY = (realNewY + aprilTagOffset)-12;
                NewX = 360 - (realNewX + 45);

                RobotLog.d("NewX" + NewX);
                RobotLog.d("NewY" + NewY);

                telemetry.addData("X reset pos", NewX);
                telemetry.addData("Y reset pos", NewY);
                telemetry.update();

                newPosition = new Vector2D(NewX, NewY);

                odometry.reset(newPosition);

                odometry.update();

            }
        }

    }

}
