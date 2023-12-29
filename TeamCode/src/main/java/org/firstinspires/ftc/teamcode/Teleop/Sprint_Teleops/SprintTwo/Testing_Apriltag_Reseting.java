package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintTwo;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class Testing_Apriltag_Reseting extends OpMode {

    Odometry odometry = new Odometry(90, 337, 90);
    Drivetrain drive = new Drivetrain();

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;

    CameraName cameraName;

    AprilTagDetection rightTag;

    private VisionPortal visionPortal;

    @Override
    public void init() {
        odometry.init(hardwareMap);
        drive.init(hardwareMap);
        initAprilTag();
    }

    @Override
    public void loop() {

        odometry.update();

        telemetryAprilTag();

        double vertical = -gamepad1.right_stick_y;
        double horizontal = gamepad1.right_stick_x * 1.5;
        double pivot = gamepad1.left_stick_x;

        drive.RF.setPower((-pivot + (vertical - horizontal)));
        drive.RB.setPower((1.15) * (-pivot + (vertical + horizontal)));
        drive.LF.setPower((pivot + (vertical + horizontal)));
        drive.LB.setPower((1.15) * (pivot + (vertical - horizontal)));

        if(gamepad1.a && !(rightTag == null)){

            odometry.update();

            if(rightTag.id == 4) {


                double NewY;

                Vector2D newPosition;

                //red side
                if(rightTag.ftcPose.x > 0){
                    NewY = 286 + (rightTag.ftcPose.x * 0.1);
                }else{
                    NewY = 286 + (rightTag.ftcPose.x * 0.1);
                }

                newPosition = new Vector2D(((rightTag.ftcPose.y * 0.1)-5.5) + 21, NewY);

                odometry.reset(newPosition);

                telemetry.addData("rightTag.ftcPose.y", rightTag.ftcPose.y);
                telemetry.addData("rightTag.ftcPose.x", rightTag.ftcPose.x);
                telemetry.update();

            }

        }

        telemetry.addData("x", odometry.X);
        telemetry.addData("y", odometry.Y);
        telemetry.addData("heading", odometry.heading);
        telemetry.update();
    }

    private void initAprilTag() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        cameraName = hardwareMap.get(CameraName.class, "resetcam");
        builder.setCamera(cameraName);


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);

        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            if (detection.id == 4 || detection.id == 9){
                // distance to apriltag from wall - odometry.Y = detection.ftcPose.x;
                rightTag = detection;
            }

        }   // end for() loop

    }

}
