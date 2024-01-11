package org.firstinspires.ftc.teamcode.Auto.OtherAuto.Test_Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueLeftBuilder;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Disabled
/**start red right*/
public class Testing_Odo_Drift extends LinearOpMode {

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.blue);

    /**hardware objects*/
    Odometry odometry = new Odometry(300, 90, 180);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    blueLeftBuilder firstPath = new blueLeftBuilder();

    blueLeftBuilder secondPath = new blueLeftBuilder();

    blueLeftBuilder thridPath = new blueLeftBuilder();

    mecanumFollower follower = new mecanumFollower();

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        while (opModeInInit()){

            odometry.update();

            telemetry.addData("X", odometry.X);
            telemetry.addData("Y", odometry.Y);
            telemetry.addData("heading", odometry.heading);
            telemetry.update();

        }

        waitForStart();

        firstPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.collect);

        secondPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.deliver);

        follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

        delivery.setGripperState(Delivery.targetGripperState.openBoth);
        delivery.updateGrippers();

        collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
        collection.updateIntakeHeight();

        follower.followPath(180, odometry, drive, collection, new Vector2D(76, 209));

        odometry.update();

        sleep(1000);

        delivery.setGripperState(Delivery.targetGripperState.closeBoth);
        delivery.updateGrippers();

        sleep(500);

        collection.setState(Collection.intakePowerState.reversed);
        collection.updateIntakeState();

        sleep(400);

        collection.setState(Collection.intakePowerState.off);
        collection.updateIntakeState();

        follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

        follower.followPath(180, odometry, drive);

        dropWhitePixels();

        while (opModeIsActive()){

            odometry.update();

            telemetry.addData("X", odometry.X);
            telemetry.addData("Y", odometry.Y);
            telemetry.addData("heading", odometry.heading);
            telemetry.update();

        }


    }

    private void initialize(){

        //init hardware
        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        collection.init(hardwareMap);

        delivery.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        odometry.update();

    }

    private void dropYellowPixel(){

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        sleep(200);

        deliverySlides.DeliverySlides(500, 0.6);

        while (deliverySlides.getCurrentposition() < 500){}

        delivery.setArmTargetState(Delivery.armState.deliverAuto);
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, gamepad1, telemetry, gamepad2);

        sleep(1500);

        delivery.setGripperState(Delivery.targetGripperState.openRight);
        delivery.updateGrippers();

        sleep(1500);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, gamepad1, telemetry, gamepad2);

        sleep(100);

        deliverySlides.DeliverySlides(0, -0.6);

        sleep(500);

        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
        collection.updateIntakeHeight();

    }

    private void dropWhitePixels(){

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        sleep(200);

        deliverySlides.DeliverySlides(700, 0.6);

        while (deliverySlides.getCurrentposition() < 680){}

        delivery.setArmTargetState(Delivery.armState.deliverAuto);
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, gamepad1, telemetry, gamepad2);

        sleep(1500);

        delivery.setGripperState(Delivery.targetGripperState.openBoth);
        delivery.updateGrippers();

        sleep(1000);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, gamepad1, telemetry, gamepad2);

        sleep(100);

        deliverySlides.DeliverySlides(0, -0.6);

        sleep(500);

        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
        collection.updateIntakeHeight();

    }

}