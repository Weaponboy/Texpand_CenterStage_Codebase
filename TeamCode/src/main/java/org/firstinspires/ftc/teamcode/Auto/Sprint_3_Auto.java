package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
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
public class Sprint_3_Auto extends LinearOpMode {

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.blue);

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 337, 90);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    redRightBuilder firstPath = new redRightBuilder();

    redRightBuilder secondPath = new redRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    boolean SlideSafetyHeight = false;

    boolean SlideSafetyBottom = false;

    //in ms
    double timePerDegreeTopPivot = 2.8;

    double smallServoTimePerDegree = 1.6;

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

        if (propPos == 1){

            firstPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload);

            secondPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.collect);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(45, odometry, drive, point = new Vector2D(229, 260), 180);

            odometry.update();

            dropYellowPixel();

            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

            follower.followPath(180, odometry, drive);

        } else if (propPos == 2) {

            firstPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.preload);

            secondPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.collect);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(90, odometry, drive, point = new Vector2D(210, 300), 180);

            odometry.update();

            dropYellowPixel();

            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

            follower.followPath(180, odometry, drive);

        } else if (propPos == 3) {

            firstPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.preload);

            secondPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.collect);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(135, odometry, drive, point = new Vector2D(238, 302), 180);

            odometry.update();

            dropYellowPixel();

            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

            follower.followPath(180, odometry, drive);

        }

    }

    private void initialize(){

        //init hardware
        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        collection.init(hardwareMap);

        delivery.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        delivery.setMainPivot(0.1);

        delivery.setSecondPivot(1);

        delivery.RotateClaw.setPosition(0.5);

        delivery.RightClaw.setPosition(clawClosed);

        delivery.LeftClaw.setPosition(clawClosed);

        collection.IntakeHeight.setPosition(intakeSafeInRobot);

        odometry.update();

        frontCam = hardwareMap.get(WebcamName.class, "frontCam");

        portal = VisionPortal.easyCreateWithDefaults(frontCam, propDetectionByAmount);

    }

    private void dropYellowPixel(){

        collection.IntakeHeight.setPosition(0.4);

        sleep(200);

        deliverySlides.DeliverySlides(250, 0.6);

        while (deliverySlides.Left_Slide.isBusy()){}

        timeToWait = (long) Math.max((Math.abs(delivery.getSecondPivotPosition()-deliverySecondPivot)*180)*timePerDegreeTopPivot, (Math.abs(delivery.getTopPivotPosition()-deliveryTopPivot)*180)*timePerDegreeTopPivot);

        delivery.setClaws(clawClosed);

        delivery.setSecondPivot(deliverySecondPivot);

        delivery.setMainPivot(deliveryTopPivot);

        delivery.RotateClaw.setPosition(rotateCollect);

        sleep(2000);

        delivery.setClaws(clawOpen);

        sleep(1000);

        timeToWait = (long) Math.max((Math.abs(delivery.getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegreeTopPivot, (Math.abs(delivery.getTopPivotPosition() - collectTopPivotPos) * 180) * timePerDegreeTopPivot);

        delivery.setClaws(clawClosed);

        delivery.setSecondPivot(collectSecondPivot);

        delivery.setMainPivot(collectTopPivotPos);

        delivery.RotateClaw.setPosition(rotateCollect);

        sleep(1000);

        deliverySlides.DeliverySlides(0, -0.6);

    }

}
