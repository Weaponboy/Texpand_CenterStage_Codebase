package org.firstinspires.ftc.teamcode.Testing.Test_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.hardware.Sensors;
import org.firstinspires.ftc.teamcode.Auto.Methods.Auto_Methods;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
@Disabled
/**start red right*/
public class TestingReset extends LinearOpMode implements Auto_Methods{

    public WebcamName frontCam;

    public VisionPortal portal;

    AprilTagProcessor aprilTag = null;

    org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.color.blue);

    /**hardware objects*/
    Odometry odometry = new Odometry(90, 23, 270);

    Drivetrain drive = new Drivetrain();

    Sensors sensors = new Sensors();

    /**pathing objects*/
    blueRightBuilder preloadPurple = new blueRightBuilder();

    blueRightBuilder preloadYellow = new blueRightBuilder();

    blueRightBuilder collect = new blueRightBuilder();

    blueRightBuilder deliver = new blueRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    blueRightBuilder lastToBackboard = new blueRightBuilder();

    boolean reset = false;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

//        preloadPurple.buildPath(blueRightBuilder.Position.center, blueRightBuilder.Section.preload);
//
//        follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);
//
//        follower.followPath(270, odometry, drive, new Vector2D(75, 163), 180);

        while (!reset){

            sensors.getDetections();

            resetOdo();

        }

//        odometry.update();
//
//        Vector2D startPos = new Vector2D(odometry.X, odometry.Y);
//
//        lastToBackboard.buildPathLine(startPos, new Vector2D(240, 90));
//
//        follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);
//
//        follower.followPath(180, odometry, drive);
//
//        odometry.update();

        while (opModeIsActive()){

            odometry.update();

            telemetry.addData("X", odometry.X);
            telemetry.addData("Y", odometry.Y);
            telemetry.addData("heading", odometry.heading);
            telemetry.update();
        }

    }

    private void initialize(){

        //init hardware
        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        init(hardwareMap);

        odometry.update();

        sensors.init(hardwareMap);

        sensors.initAprilTag(telemetry, true);

    }

    public void resetOdo(){

        if (!(sensors.rightTag == null)){

            if (sensors.rightTag.id == 4 || sensors.rightTag.id == 5 || sensors.rightTag.id == 6){

                double NewY;
                double NewX;

                double aprilTagOffset;

                Vector2D newPosition;

                if (sensors.rightTag.id == 4){
                    aprilTagOffset = getRealCoords(75);
                }else if (sensors.rightTag.id == 5){
                    aprilTagOffset = getRealCoords(90);
                }else{
                    aprilTagOffset = getRealCoords(105);
                }

                double realNewY = (sensors.rightTag.ftcPose.x * 0.1);
                double realNewX = (sensors.rightTag.ftcPose.y * 0.1);

                NewY = aprilTagOffset + realNewY;
                NewX = 360 - (realNewX + 45);

                newPosition = new Vector2D(NewX, NewY - 12.5);

                double heading = 180 + (-(sensors.rightTag.ftcPose.yaw));

                odometry.reset(newPosition, heading);

                reset = true;

                telemetry.addData("rightTag.ftcPose.yaw", sensors.rightTag.ftcPose.yaw);
                telemetry.addData("rightTag.ftcPose.y", sensors.rightTag.ftcPose.y);
                telemetry.addData("realNewX", sensors.rightTag.ftcPose.x);
                telemetry.addData("X reset pos", newPosition.getX());
                telemetry.addData("Y reset pos", newPosition.getY());

            }
        }

    }

}
