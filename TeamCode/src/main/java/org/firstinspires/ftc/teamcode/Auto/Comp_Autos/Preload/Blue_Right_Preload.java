package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Preload;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Sensors;
import org.firstinspires.ftc.teamcode.hardware.Method_Interfaces.Auto_Methods;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Blue_Right_Preload", group = "Preload")
/**start red right*/
public class Blue_Right_Preload extends LinearOpMode implements Auto_Methods{

    public WebcamName frontCam;

    public VisionPortal portal;

    AprilTagProcessor aprilTag = null;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.blue);

    /**hardware objects*/
    Odometry odometry = new Odometry(90, 23, 270);

    Drivetrain drive = new Drivetrain();

    Sensors sensors = new Sensors();

    /**pathing objects*/
    blueRightBuilder preloadPurple = new blueRightBuilder();

    blueRightBuilder preloadYellow = new blueRightBuilder();

    blueRightBuilder collect = new blueRightBuilder();

    blueRightBuilder deliver = new blueRightBuilder();

    blueRightBuilder lastToBackboard = new blueRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    List<LynxModule> allHubs;

    boolean reset = false;

    int counter;

    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        if (propPos == 1){

            preloadPurple.buildPath(blueRightBuilder.Position.left, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.purple);

            preloadYellow.buildPath(blueRightBuilder.Position.left, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.yellow);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(90, 75), 180);

            odometry.update();

            follower.setPath(preloadYellow.followablePath, preloadYellow.pathingVelocity);

            follower.followPath(180, odometry, drive);

            elapsedTime.reset();

            counter = 0;

            while (!reset){

                sensors.getDetections();

                odometry.update();

                resetOdo();

            }

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            lastToBackboard.buildPathLine(startPos, new Vector2D(306, 80));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");

            dropYellowPixel();

        } else if (propPos == 2) {

            preloadPurple.buildPath(blueRightBuilder.Position.center, blueRightBuilder.Section.preload);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(75, 163), 180, allHubs);

            elapsedTime.reset();

            counter = 0;

            while (!reset){

                sensors.getDetections();

                odometry.update();

                resetOdo();

            }

//            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);
//
//            lastToBackboard.buildPathLine(startPos, new Vector2D(306, 100));
//
//            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);
//
//            follower.followPath(180, odometry, drive, "yes");
//
//            dropYellowPixel();

        } else if (propPos == 3) {

            preloadPurple.buildPath(blueRightBuilder.Position.right, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.purple);

            preloadYellow.buildPath(blueRightBuilder.Position.right, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.yellow);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(300, odometry, drive);

            odometry.update();

            follower.setPath(preloadYellow.followablePath, preloadYellow.pathingVelocity);

            follower.followPath(270, odometry, drive, new Vector2D(97, 171), 180);

            elapsedTime.reset();

            counter = 0;

            while (!reset){

                sensors.getDetections();

                odometry.update();

                resetOdo();

            }

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            lastToBackboard.buildPathLine(startPos, new Vector2D(306, 120));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");

            dropYellowPixel();

        }
    }

    private void initialize(){

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //init hardware
        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        init(hardwareMap);

        odometry.update();

        sensors.init(hardwareMap);

        sensors.initAprilTag(telemetry, false);

    }

    public void resetOdo(){

        if (!(sensors.rightTag == null)){

            if (sensors.rightTag.id == 1 || sensors.rightTag.id == 2 || sensors.rightTag.id == 3){

                counter++;

                double NewY;
                double NewX;

                double aprilTagOffset;

                Vector2D newPosition;

                if (sensors.rightTag.id == 1){
                    aprilTagOffset = getRealCoords(75);
                }else if (sensors.rightTag.id == 2){
                    aprilTagOffset = getRealCoords(90);
                }else{
                    aprilTagOffset = getRealCoords(105);
                }

                double heading = odometry.getIMUHeading();

                double realNewX = (sensors.rightTag.ftcPose.y * 0.1);
                double realNewY = (sensors.rightTag.ftcPose.x * 0.1);

                NewY = (realNewY + aprilTagOffset)-12;
                NewX = 360 - (realNewX + 45);

                newPosition = new Vector2D(NewX, NewY);

                odometry.reset(newPosition, heading);

                if (counter == 4){
                    reset = true;
                }

                telemetry.addData("rightTag.ftcPose.yaw", Math.toDegrees(sensors.rightTag.ftcPose.yaw));
                telemetry.addData("rightTag.ftcPose.y", sensors.rightTag.ftcPose.y);
                telemetry.addData("realNewX", sensors.rightTag.ftcPose.x);
                telemetry.addData("X reset pos", newPosition.getX());
                telemetry.addData("Y reset pos", newPosition.getY());
                telemetry.update();

            }
        }

    }

}
