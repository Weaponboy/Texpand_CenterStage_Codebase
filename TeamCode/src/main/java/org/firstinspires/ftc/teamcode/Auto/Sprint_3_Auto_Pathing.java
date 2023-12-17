package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Old.pathBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

@Autonomous
/**start red right*/
public class Sprint_3_Auto_Pathing extends LinearOpMode {

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

        //build path to drop purple pixel
        firstPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.preload);

        //give path to follower
        follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

        odometry.update();

        waitForStart();

        follower.followPath(true, 90, false, odometry, drive, telemetry);

        drive.setAllPower(0.4);

        sleep(400);

        odometry.update();

        secondPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.collect);

        follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

        odometry.update();

        follower.followPath(true, 180, false, odometry, drive, telemetry);

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

        sleep(500);

    }

}
