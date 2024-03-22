package org.firstinspires.ftc.teamcode.Auto.OtherAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.GenMethods;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;
import org.firstinspires.ftc.teamcode.hardware._.Sensors;

@TeleOp
public class TestingDelivery extends LinearOpMode {

    GenMethods preloadPaths = new GenMethods();
    GenMethods preloadPaths2 = new GenMethods();

    Odometry odometry = new Odometry(210, 23, 270);

    Drivetrain drive = new Drivetrain();

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Sensors sensors = new Sensors();

    mecanumFollower follower = new mecanumFollower();

    boolean lockIn = false;

    enum Auto{
        left,
        right,
    }

    Auto auto = Auto.right;

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);

        odometry.init(hardwareMap);

        delivery.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        sensors.init(hardwareMap);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();
//
//        while(opModeInInit()){
//
//            if (!lockIn) {
//
//                telemetry.addData("side activated", auto);
//                telemetry.addData("press y for left", "");
//                telemetry.addData("press a for right", "");
//                telemetry.addData("press x to lock in!!!!", "");
//                telemetry.update();
//
//                if (gamepad1.a){
//                    auto = Auto.right;
//                }else if (gamepad1.y) {
//                    auto = Auto.left;
//                }else if (gamepad1.x) {
//                    lockIn = true;
//                }
//            }
//
//        }

        waitForStart();

        boolean pathing = true;

        preloadPaths.twoPoints(new Vector2D(210,23), new Vector2D(215, 55), true);

        follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);

        deliverySlides.DeliverySlides(1000, 1);

        while (pathing || Math.abs(odometry.heading - 180) > 2){
            pathing = follower.followPathAuto(180, odometry, drive);
        }

        drive.setAllPower(0);

        delivery.setArmTargetState(Delivery.armState.deliverAuto);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

        while (!(delivery.getArmState() == Delivery.armState.deliverAuto)){
            delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
        }

        drive.setAllPower(0);

        preloadPaths2.twoPoints(new Vector2D(215,55), new Vector2D(315, 55), true);

        follower.setPath(preloadPaths2.followablePath, preloadPaths2.pathingVelocity);

        boolean drop = false;

        pathing = true;

        while (!drop){

            if (!pathing){
                drop = true;
            }
            pathing = follower.followPathAuto(180, odometry, drive);

            if(sensors.armSensor.isPressed()){
                drive.setAllPower(0.2);
                drop = true;
            }

        }

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(100);

        drive.setAllPower(0);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

        while (!(delivery.getArmState() == Delivery.armState.collect)){
            delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
            drive.setAllPower(0.4);
        }

        drive.setAllPower(0);

//        delivery.setArmTargetState(Delivery.armState.collect);
//        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
//
//        while (delivery.getArmState() == Delivery.armState.deliverAuto){
//            delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
//        }

//        preloadPaths.twoPoints(new Vector2D(210,50), new Vector2D(305, 50), true);
//
//        follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);

//        if (auto == Auto.right){
//
//            preloadPaths.twoPoints(new Vector2D(210,23), new Vector2D(312, 143), true);
//
//            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
//
//            deliverySlides.DeliverySlides(1000, 1);
//
//            while (Math.abs(312 - odometry.X) > 1.4 && Math.abs(143 - odometry.Y) > 1.4){
//                follower.followPathAuto(135, odometry, drive);
//
//                if (deliverySlides.getCurrentposition() > 150) {
//                    delivery.setArmTargetState(Delivery.armState.deliverAuto);
//                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
//                }
//            }
//
//            drive.setAllPower(0);
//
//            while (deliverySlides.getCurrentposition() < 950){
//
//            }
//
//            drive.setAllPower(0);
//
//            delivery.setArmTargetState(Delivery.armState.droppingWhites);
//            delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
//
//
//            while (!(delivery.getArmState() == Delivery.armState.readyToDrop)){
//                delivery.updateArm(deliverySlides.getCurrentposition(), sensors.armSensor.isPressed(), Delivery.PixelsAuto.backboardRight, odometry);
//            }
//
//        } else if (auto == Auto.left) {
//
//            preloadPaths.twoPoints(new Vector2D(210,23), new Vector2D(305, 50), true);
//
//            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
//
////            deliverySlides.DeliverySlides(1000, 1);
//
//            while (Math.abs(210 - odometry.X) > 1.4 && Math.abs(50 - odometry.Y) > 1.4){
//                follower.followPathAuto(180, odometry, drive);
////                if (deliverySlides.getCurrentposition() > 150) {
////                    delivery.setArmTargetState(Delivery.armState.deliverAuto);
////                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
////                }
//            }
//
//            drive.setAllPower(0);
//
//            preloadPaths.twoPoints(new Vector2D(210,50), new Vector2D(305, 50), true);
//
//            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
//
////            while (deliverySlides.getCurrentposition() < 950){
////
////            }
////
////            drive.setAllPower(0);
////
////            delivery.setArmTargetState(Delivery.armState.droppingWhites);
////            delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
////
////
////            while (!(delivery.getArmState() == Delivery.armState.readyToDrop)){
////                delivery.updateArm(deliverySlides.getCurrentposition(), sensors.armSensor.isPressed(), Delivery.PixelsAuto.backboardLeft, odometry);
////            }
//        }
//
//        telemetry.addData("sensor", sensors.armSensor.isPressed());
//        telemetry.update();
//
//        delivery.setGripperState(Delivery.GripperState.open);
//        delivery.updateGrippers();
//
//        telemetry.addData("sensor", sensors.armSensor.isPressed());
//        telemetry.update();
//
//        sleep(500);
//
//        delivery.setArmTargetState(Delivery.armState.collect);
//        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
//

        while (opModeIsActive()){

        }
    }

}
