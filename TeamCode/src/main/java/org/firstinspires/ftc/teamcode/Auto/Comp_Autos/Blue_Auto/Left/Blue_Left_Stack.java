package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Blue_Auto.Left;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueLeftBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;
import org.firstinspires.ftc.teamcode.hardware.Method_Interfaces.Auto_Methods;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Objects;

@Autonomous
/**start red right*/
public class Blue_Left_Stack extends LinearOpMode{

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

    blueLeftBuilder posThreeExtra = new blueLeftBuilder();

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

            //close vision portal
            portal.close();

            //build paths
            firstPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.preload, redRightBuilder.pathSplit.first);

            posThreeExtra.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.preload, redRightBuilder.pathSplit.second);

            //follow first path
            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(210, 70), 0);

            odometry.update();

            //follow second path
            follower.setPath(posThreeExtra.followablePath, posThreeExtra.pathingVelocity);

            follower.followPath(0, odometry, drive, new Vector2D(250, 90), 180);

            odometry.update();

            dropYellowPixel();

            //build stack paths
            secondPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.collect);

            thridPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.deliver);

            stackPixels();

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

    public void stackPixels() {

        follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

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

        follower.setPath(thridPath.followablePath, thridPath.pathingVelocity);

        follower.followPath(180, odometry, drive);

        dropWhitePixels();

    }

    public void dropYellowPixel() {

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        sleep(200);

        deliverySlides.DeliverySlides(500, 0.6);

        while (deliverySlides.getCurrentposition() < 500){}

        delivery.setArmTargetState(Delivery.armState.deliverAuto);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(1500);

        delivery.setGripperState(Delivery.targetGripperState.openRight);
        delivery.updateGrippers();

        sleep(1500);

        delivery.setArmTargetState(Delivery.armState.collect);
        delivery.updateArm(deliverySlides.getCurrentposition());

        sleep(100);

        deliverySlides.DeliverySlides(0, -0.6);

        sleep(500);

        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
        collection.updateIntakeHeight();

    }

    public void dropWhitePixels(){

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        sleep(200);

        deliverySlides.DeliverySlides(700, 0.6);

        while (deliverySlides.getCurrentposition() < 680){

            if (deliverySlides.getCurrentposition() > 150){
                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition());
            }

        }

        boolean armInPosition = false;

        while (!armInPosition){

            if (Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.deliverAuto) {
                armInPosition = true;
            }
            delivery.updateArm(deliverySlides.getCurrentposition());
        }

        sleep(200);

        delivery.setGripperState(Delivery.targetGripperState.openBoth);
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

}
