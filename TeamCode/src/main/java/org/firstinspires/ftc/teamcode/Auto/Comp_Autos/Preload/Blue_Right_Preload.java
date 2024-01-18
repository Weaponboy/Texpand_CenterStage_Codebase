package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Preload;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
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
public class Blue_Right_Preload extends LinearOpMode implements Auto_Methods{

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.blue);

    /**hardware objects*/
    Odometry odometry = new Odometry(90, 23, 270);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    blueRightBuilder preloadPurple = new blueRightBuilder();

    blueRightBuilder preloadYellow = new blueRightBuilder();

    blueRightBuilder collect = new blueRightBuilder();

    blueRightBuilder deliver = new blueRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        propPos = 3;

        if (propPos == 1){

            portal.close();

            preloadPurple.buildPath(blueRightBuilder.Position.left, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.purple);

            preloadYellow.buildPath(blueRightBuilder.Position.left, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.yellow);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(90, 75), 180);

            odometry.update();

            follower.setPath(preloadYellow.followablePath, preloadYellow.pathingVelocity);

            follower.followPath(180, odometry, drive);

            dropYellowPixel();

        } else if (propPos == 2) {

            portal.close();

            preloadPurple.buildPath(blueRightBuilder.Position.center, blueRightBuilder.Section.preload);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(75, 163), 180);

            odometry.update();

            dropYellowPixel();

        } else if (propPos == 3) {

            portal.close();

            preloadPurple.buildPath(blueRightBuilder.Position.right, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.purple);

            preloadYellow.buildPath(blueRightBuilder.Position.right, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.yellow);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(300, odometry, drive);

            odometry.update();

            follower.setPath(preloadYellow.followablePath, preloadYellow.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(102, 174), 180);

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
