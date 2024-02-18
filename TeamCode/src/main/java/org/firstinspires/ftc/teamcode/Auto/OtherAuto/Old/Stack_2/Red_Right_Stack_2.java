package org.firstinspires.ftc.teamcode.Auto.OtherAuto.Old.Stack_2;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;
import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Methods.Auto_Methods;
import org.firstinspires.ftc.vision.VisionPortal;
@Disabled
@Autonomous(name = "Red_Right_Stack+2", group = "Stack 2+2")
/**start red right*/
public class Red_Right_Stack_2 extends LinearOpMode implements Auto_Methods {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.red);

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 337, 90);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    redRightBuilder firstPath = new redRightBuilder();

    redRightBuilder secondPath = new redRightBuilder();

    redRightBuilder thirdPath = new redRightBuilder();

    blueRightBuilder lastToBackboard = new blueRightBuilder();

    mecanumFollower follower = new mecanumFollower();


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        if (propPos == 1){

            portal.close();

            firstPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload, redRightBuilder.pathSplit.first);

            secondPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload, redRightBuilder.pathSplit.second);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(90, odometry, drive, point = new Vector2D(210, 290), 0);

            odometry.update();

            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

            follower.followPath(0, odometry, drive, point = new Vector2D(250, 270), 180);

            odometry.update();

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            dropYellowPixel();

            lastToBackboard.buildPathLine(startPos, new Vector2D(290, 330));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");

        } else if (propPos == 2) {

            portal.close();

            firstPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.preload);
//
//            secondPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.collect);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(90, odometry, drive, point = new Vector2D(236, 300), 180);

            odometry.update();

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            dropYellowPixel();

            lastToBackboard.buildPathLine(startPos, new Vector2D(290, 330));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");


        } else if (propPos == 3) {

            portal.close();

            firstPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.preload);
//            secondPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.collect);
//
//            thridPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.deliver);
            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(120, odometry, drive, point = new Vector2D(250, 314), 180);

            odometry.update();

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            dropYellowPixel();

            lastToBackboard.buildPathLine(startPos, new Vector2D(290, 330));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");



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

}
