package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Preload;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Red_Points_Overlap;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redLeftBuilder;
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
public class Red_Left_Preload extends LinearOpMode implements Auto_Methods {

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.red);

    /**hardware objects*/
    Odometry odometry = new Odometry(90, 337, 90);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    redLeftBuilder preloadPurple = new redLeftBuilder();

    redLeftBuilder preloadYellow = new redLeftBuilder();

    redLeftBuilder collect = new redLeftBuilder();

    redLeftBuilder deliver = new redLeftBuilder();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        propPos = 3;

        if (propPos == 1){

            portal.close();

            preloadPurple.buildPath(redLeftBuilder.Position.left, redLeftBuilder.Section.preload, redLeftBuilder.pixelColor.purple);

            preloadYellow.buildPath(redLeftBuilder.Position.left, redLeftBuilder.Section.preload, redLeftBuilder.pixelColor.yellow);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(90, 285), 180);

            odometry.update();

            follower.setPath(preloadYellow.followablePath, preloadYellow.pathingVelocity);

            follower.followPath(180, odometry, drive);

            dropYellowPixel();

        } else if (propPos == 2) {

            portal.close();

            preloadPurple.buildPath(redLeftBuilder.Position.center, redLeftBuilder.Section.preload);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(75, 197), 180);

            odometry.update();

            dropYellowPixel();

        } else if (propPos == 3) {

            portal.close();

            preloadPurple.buildPath(redLeftBuilder.Position.right, redLeftBuilder.Section.preload, redLeftBuilder.pixelColor.purple);

            preloadYellow.buildPath(redLeftBuilder.Position.right, redLeftBuilder.Section.collect, redLeftBuilder.pixelColor.yellow);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(300, odometry, drive);

            odometry.update();

            follower.setPath(preloadYellow.followablePath, preloadYellow.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(102, 186), 180);

            dropYellowPixel();

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

}
