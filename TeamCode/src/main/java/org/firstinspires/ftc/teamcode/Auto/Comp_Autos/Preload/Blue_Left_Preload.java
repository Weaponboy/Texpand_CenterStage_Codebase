package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Preload;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueLeftBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue_Left_Preload", group = "Preload")
/**start red right*/
public class Blue_Left_Preload extends LinearOpMode implements Auto_Methods {

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.blue);

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 23, 270);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    blueLeftBuilder firstPath = new blueLeftBuilder();

    blueLeftBuilder secondPath = new blueLeftBuilder();

    blueLeftBuilder thirdPath = new blueLeftBuilder();

    blueRightBuilder lastToBackboard = new blueRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        if (propPos == 1){

            portal.close();

            firstPath.buildPath(blueLeftBuilder.Position.left, blueLeftBuilder.Section.preload);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            follower.followPath(240, odometry, drive, new Vector2D(250, 46), 180);

            odometry.update();

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            dropYellowPixel();

            lastToBackboard.buildPathLine(startPos, new Vector2D(290, 30));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");

        } else if (propPos == 2) {

            portal.close();

            firstPath.buildPath(blueLeftBuilder.Position.center, blueLeftBuilder.Section.preload);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            //change target heading after dropping the purple pixel
            Vector2D point;
            follower.followPath(270, odometry, drive, point = new Vector2D(236, 60), 180);

            odometry.update();

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            dropYellowPixel();

            lastToBackboard.buildPathLine(startPos, new Vector2D(290, 30));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");

        } else if (propPos == 3) {

            portal.close();

            firstPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.preload, redRightBuilder.pathSplit.first);

            secondPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.preload, redRightBuilder.pathSplit.second);

            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(210, 70), 0);

            odometry.update();

            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);

            follower.followPath(0, odometry, drive, new Vector2D(250, 90), 180);

            odometry.update();

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            dropYellowPixel();

            lastToBackboard.buildPathLine(startPos, new Vector2D(290, 30));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");

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
