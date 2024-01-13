package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Stack;

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
public class Blue_Left_Stack extends LinearOpMode implements Auto_Methods{

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


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();
        propPos = 3;

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

        init(hardwareMap);

        odometry.update();

        frontCam = hardwareMap.get(WebcamName.class, "frontcam");

        portal = VisionPortal.easyCreateWithDefaults(frontCam, propDetectionByAmount);

    }

    public void stackPixels() throws InterruptedException {

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

}