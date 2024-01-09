package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueLeftBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
/**start red right*/
public class BlueLeftAuto extends LinearOpMode {

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.blue);

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 23, 270);

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

        waitForStart();

        if (propPos == 1){

            portal.close();

            firstPath.buildPath(blueLeftBuilder.Position.left, blueLeftBuilder.Section.preload);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(240, odometry, drive, point = new Vector2D(250, 46), 180);

            odometry.update();

            dropYellowPixel();

        } else if (propPos == 2) {

            portal.close();

            firstPath.buildPath(blueLeftBuilder.Position.center, blueLeftBuilder.Section.preload);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(270, odometry, drive, point = new Vector2D(236, 60), 180);

            odometry.update();

            dropYellowPixel();

        } else if (propPos == 3) {

            portal.close();

            firstPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.preload, redRightBuilder.pathSplit.first);

            secondPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.preload, redRightBuilder.pathSplit.second);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(270, odometry, drive, point = new Vector2D(210, 70), 0);

            odometry.update();

            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

            follower.followPath(0, odometry, drive, point = new Vector2D(250, 90), 180);

            odometry.update();

            dropYellowPixel();

//
//            delivery.setGripperState(Delivery.targetGripperState.openBoth);
//            delivery.updateGrippers();
//
//            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
//            collection.updateIntakeHeight();
//
//            collection.setState(Collection.intakePowerState.on);
//            collection.updateIntakeState();
//
//            sleep(1500);
//
//            collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
//            collection.updateIntakeHeight();
//
//            sleep(1500);
//
//            collection.setState(Collection.intakePowerState.off);
//            collection.updateIntakeState();
//
//            delivery.setGripperState(Delivery.targetGripperState.closeBoth);
//            delivery.updateGrippers();
//
//            sleep(500);
//
//            collection.setState(Collection.intakePowerState.reversed);
//            collection.updateIntakeState();
//
//            sleep(200);
//
//            collection.setState(Collection.intakePowerState.off);
//            collection.updateIntakeState();
//
//            follower.setPath(thridPath.followablePath, thridPath.pathingVelocity);
//
//            follower.followPath(180, odometry, drive);
//
//            dropWhitePixels();
//
//            sleep(200);

        }

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

        frontCam = hardwareMap.get(WebcamName.class, "frontcam");

        portal = VisionPortal.easyCreateWithDefaults(frontCam, propDetectionByAmount);

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
