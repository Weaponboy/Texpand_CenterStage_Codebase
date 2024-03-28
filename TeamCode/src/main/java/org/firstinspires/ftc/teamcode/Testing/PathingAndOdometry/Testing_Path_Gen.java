package org.firstinspires.ftc.teamcode.Testing.PathingAndOdometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Disabled
/**start red right*/
public class Testing_Path_Gen extends LinearOpMode{

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.color.blue);

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


        waitForStart();

        preloadPurple.buildPath(blueRightBuilder.Position.right, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.purple);

        preloadYellow.buildPath(blueRightBuilder.Position.left, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.yellow);

        while (opModeIsActive()){
            dashboardTelemetry.addData("target Point", preloadYellow.followablePath.get(preloadYellow.followablePath.size()-1));
            dashboardTelemetry.update();
        }

    }

}
