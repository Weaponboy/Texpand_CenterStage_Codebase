package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Stack;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

@Autonomous
/**start red right*/
public class Red_Right_Stack extends LinearOpMode implements Auto_Methods {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(dashboardTelemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.red);

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 337, 90);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    redRightBuilder firstPath = new redRightBuilder();

    redRightBuilder secondPath = new redRightBuilder();

    redRightBuilder thirdPath = new redRightBuilder();

    redRightBuilder posOneExtra = new redRightBuilder();

    mecanumFollower follower = new mecanumFollower();


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        if (propPos == 1){

            portal.close();

            firstPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload, redRightBuilder.pathSplit.first);

            posOneExtra.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload, redRightBuilder.pathSplit.second);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            follower.followPath(90, odometry, drive, new Vector2D(210, 290), 0);

            odometry.update();

            follower.setPath(posOneExtra.followablePath, posOneExtra.pathingVelocity);

            follower.followPath(0, odometry, drive, new Vector2D(250, 270), 180);

            odometry.update();

            dropYellowPixel();

            secondPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.collect);

            thirdPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.deliver);

            dropWhitePixels();

        } else if (propPos == 2) {

            portal.close();

            firstPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.preload);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            follower.followPath(90, odometry, drive, new Vector2D(236, 300), 180);

            odometry.update();

            dropYellowPixel();

            secondPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.collect);

            thirdPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.deliver);

            dropWhitePixels();

        } else if (propPos == 3) {

            portal.close();

            firstPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.preload);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            follower.followPath(120, odometry, drive, new Vector2D(250, 314), 180);

            odometry.update();

            dropYellowPixel();

            secondPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.collect);

            thirdPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.deliver);

            dropWhitePixels();

        }


    }

    private void initialize(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
        collection.updateIntakeHeight();

        follower.followPath(180, odometry, drive, collection, new Vector2D(76, 209));

        odometry.update();

        sleep(1000);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        sleep(500);

        collection.setState(Collection.intakePowerState.reversed);
        collection.updateIntakeState();

        sleep(400);

        collection.setState(Collection.intakePowerState.off);
        collection.updateIntakeState();

        follower.setPath(thirdPath.followablePath, thirdPath.pathingVelocity);

        follower.followPath(180, odometry, drive);

        dropWhitePixels();

    }
}
