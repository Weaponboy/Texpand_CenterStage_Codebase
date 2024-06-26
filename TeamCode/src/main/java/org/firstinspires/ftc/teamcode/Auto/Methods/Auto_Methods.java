package org.firstinspires.ftc.teamcode.Auto.Methods;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.hardware.Sensors;

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

    default void dropYellowPixel(Delivery.PixelsAuto pixelPlacement, Odometry odometry) throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, pixelPlacement, odometry);

        while (!(delivery.getArmState() == Delivery.armState.readyToDrop)){
            delivery.updateArm(deliverySlides.getCurrentposition(), sensors.armSensor.isPressed(), pixelPlacement, odometry);
        }

        if (!(sensors.armSensor.isPressed())){
            delivery.ArmExtension.setPosition(delivery.getExtensionSetPoint(pixelPlacement)-0.1);
            sleep(200);
        }

        sleep(200);

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(200);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, pixelPlacement, odometry);

        deliverySlides.DeliverySlides(0, -0.5);

        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

    }

    default void dropYellowPixelWait(Delivery.PixelsAuto pixelPlacement, Odometry odometry) throws InterruptedException {

        delivery.setArmTargetState(Delivery.armState.delivery);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, pixelPlacement, odometry);

        while (!(delivery.getArmState() == Delivery.armState.readyToDrop)){
            delivery.updateArm(deliverySlides.getCurrentposition(), false, pixelPlacement, odometry);
        }

        sleep(400);

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(200);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, pixelPlacement, odometry);

        deliverySlides.DeliverySlides(0, -0.5);

        while (deliverySlides.getCurrentposition() > 20){
            delivery.updateArm(deliverySlides.getCurrentposition(), false,  pixelPlacement, odometry);
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
