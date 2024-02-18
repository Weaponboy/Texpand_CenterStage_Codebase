package org.firstinspires.ftc.teamcode.Auto.OtherAuto.Old.Stack_2;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redLeftBuilder;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;
import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Methods.Auto_Methods;
@Disabled
@Autonomous(name = "Red_Left_Stack+2", group = "Stack 2+2")
/**start red right*/
public class Red_Left_Stack_2 extends LinearOpMode implements Auto_Methods {

    public WebcamName frontCam;

    /**hardware objects*/
    Odometry odometry = new Odometry(90, 337, 90);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    redLeftBuilder preloadPurple = new redLeftBuilder();

    redLeftBuilder preloadYellow = new redLeftBuilder();

    redLeftBuilder collect = new redLeftBuilder();

    redLeftBuilder deliver = new redLeftBuilder();

    blueRightBuilder lastToBackboard = new blueRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    boolean reset = false;

    int counter;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        if (propPos == 3){

            preloadPurple.buildPath(redLeftBuilder.Position.right, redLeftBuilder.Section.preload, redLeftBuilder.pixelColor.purple);

            preloadYellow.buildPath(redLeftBuilder.Position.right, redLeftBuilder.Section.preload, redLeftBuilder.pixelColor.yellow);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(90, odometry, drive, new Vector2D(90, 285), 180);

            odometry.update();

            follower.setPath(preloadYellow.followablePath, preloadYellow.pathingVelocity);

            follower.followPath(180, odometry, drive);

            counter = 0;

            while (!reset){

                sensors.getDetections();

                odometry.update();

                resetOdo();

            }

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            lastToBackboard.buildPathLine(startPos, new Vector2D(308, 295));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");

            dropYellowPixel();

        } else if (propPos == 2) {

            preloadPurple.buildPath(redLeftBuilder.Position.center, redLeftBuilder.Section.preload);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(90, odometry, drive, new Vector2D(75, 197), 180);

            odometry.update();

            counter = 0;

            while (!reset){

                sensors.getDetections();

                odometry.update();

                resetOdo();

            }

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            lastToBackboard.buildPathLine(startPos, new Vector2D(308, 265));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");

            dropYellowPixel();

        } else if (propPos == 1) {

            preloadPurple.buildPath(redLeftBuilder.Position.left, redLeftBuilder.Section.preload, redLeftBuilder.pixelColor.purple);

            preloadYellow.buildPath(redLeftBuilder.Position.left, redLeftBuilder.Section.preload, redLeftBuilder.pixelColor.yellow);

            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

            follower.followPath(60, odometry, drive);

            odometry.update();

            follower.setPath(preloadYellow.followablePath, preloadYellow.pathingVelocity);

            follower.followPath(90, odometry, drive, new Vector2D(97, 189), 180);

            counter = 0;

            while (!reset){

                sensors.getDetections();

                odometry.update();

                resetOdo();

            }

            Vector2D startPos = new Vector2D(odometry.X, odometry.Y);

            lastToBackboard.buildPathLine(startPos, new Vector2D(308, 250));

            follower.setPath(lastToBackboard.followablePath, lastToBackboard.pathingVelocity);

            follower.followPath(180, odometry, drive, "yes");

            dropYellowPixel();

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

                counter++;

                double NewY;
                double NewX;

                double aprilTagOffset;

                Vector2D newPosition;

                if (sensors.rightTag.id == 4){
                    aprilTagOffset = getRealCoords(255);
                }else if (sensors.rightTag.id == 5){
                    aprilTagOffset = getRealCoords(270);
                }else{
                    aprilTagOffset = getRealCoords(285);
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
