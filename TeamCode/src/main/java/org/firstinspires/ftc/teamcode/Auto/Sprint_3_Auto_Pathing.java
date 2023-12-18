package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
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

        secondPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.collect);

        //give path to follower
        follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

        odometry.update();

        waitForStart();

        //change target heading after dropping the purple pixel
        Vector2D point;
        follower.followPath(90, odometry, drive, point = new Vector2D(236, 302), 180);

        odometry.update();

        follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

        odometry.update();

        follower.followPath(true, 180, false, odometry, drive, telemetry);

    }

}
