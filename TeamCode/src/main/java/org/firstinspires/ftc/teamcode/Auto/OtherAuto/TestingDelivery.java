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

    Odometry odometry = new Odometry(210, 23, 270);

    Drivetrain drive = new Drivetrain();

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Sensors sensors = new Sensors();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);

        odometry.init(hardwareMap);

        delivery.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        sensors.init(hardwareMap);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        preloadPaths.twoPoints(new Vector2D(210,23), new Vector2D(312, 143), true);

        follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);

        waitForStart();

        deliverySlides.DeliverySlides(1000, 1);

        while (Math.abs(312 - odometry.X) > 1.4 && Math.abs(143 - odometry.Y) > 1.4){
            follower.followPathAuto(135, odometry, drive);
        }

        while (deliverySlides.getCurrentposition() < 950){

        }

        drive.setAllPower(0);

        delivery.setArmTargetState(Delivery.armState.droppingWhites);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);


        while (!(delivery.getArmState() == Delivery.armState.readyToDrop)){
            delivery.updateArm(deliverySlides.getCurrentposition(), sensors.armSensor.isPressed(), Delivery.PixelsAuto.whiteBlue, odometry);
        }

        telemetry.addData("sensor", sensors.armSensor.isPressed());
        telemetry.update();

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        telemetry.addData("sensor", sensors.armSensor.isPressed());
        telemetry.update();

        sleep(500);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);


        while (opModeIsActive()){

        }

    }

}
