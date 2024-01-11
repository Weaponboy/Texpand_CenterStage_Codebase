package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Blue_Auto.Right;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
/**start red right*/
public class Blue_Right_Preload extends LinearOpMode {

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.red);

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 337, 90);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    redRightBuilder firstPath = new redRightBuilder();

    redRightBuilder secondPath = new redRightBuilder();

    redRightBuilder thridPath = new redRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    boolean SlideSafetyHeight = false;

    boolean SlideSafetyBottom = false;

    //in ms
    double timePerDegreeTopPivot = 6;

    double smallServoTimePerDegree = 6;

    double collectTopPivotPos = 0.1;
    double deliveryTopPivot = 1;
    double safeTopPivot = 0.3;

    double avoidIntakeSecondPivot = 0.8;
    double collectSecondPivot = 1;
    double deliverySecondPivot = 0.3;

    double clawOpen = 0.5;
    double clawClosed = 0;

    double rotateCollect = 0.5;
    double rotateRight = 1;

    double intakeSafeInRobot = 0.6;
    double intakeCollect = 0.15;

    long timeToWait;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        propPos = 3;

        if (propPos == 1){

            portal.close();

            firstPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload);

            secondPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.collect);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(45, odometry, drive, point = new Vector2D(229, 260), 180);

            odometry.update();

//            dropYellowPixel();

            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

            follower.followPath(180, odometry, drive);

        } else if (propPos == 2) {

            portal.close();

            firstPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.preload);

            secondPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.collect);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(90, odometry, drive, point = new Vector2D(210, 300), 180);

            odometry.update();

//            dropYellowPixel();

            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

            follower.followPath(180, odometry, drive);

        } else if (propPos == 3) {

            portal.close();

            firstPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.preload);

            secondPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.collect);

            thridPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.deliver);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(130, odometry, drive, point = new Vector2D(250, 314), 180);

            odometry.update();

            dropYellowPixel();

            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

            follower.followPath(180, odometry, drive);

            follower.setPath(thridPath.followablePath, thridPath.pathingVelocity);

            follower.followPath(180, odometry, drive);
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

        sleep(1000);

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
