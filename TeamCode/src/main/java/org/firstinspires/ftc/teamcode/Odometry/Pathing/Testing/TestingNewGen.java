package org.firstinspires.ftc.teamcode.Odometry.Pathing.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.robotPos;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Old.pathBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Enums.whatPath;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;

@TeleOp
@Disabled
public class TestingNewGen extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    pathBuilder pathFirst = new pathBuilder();

    pathBuilder pathSecond = new pathBuilder();

    Odometry odometry = new Odometry(93,23,270);

    Drivetrain drive = new Drivetrain();

    Vector2D robotPosition = new Vector2D();

    ElapsedTime elapsedTime = new ElapsedTime();

    robotPos botFullPos = new robotPos();

    mecanumFollower follower = new mecanumFollower();

    double lastLoopTime;

    double loopTime;

    double Heading = 0;

    boolean busyPathing;

    int counter;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        //build path
        pathFirst.buildPath(whatPath.testCurve);

        follower.setPath(pathFirst.followablePath, pathFirst.pathingVelocity);

        odometry.update();

        waitForStart();

        follower.followPath(true, 180, false, odometry, drive, telemetry);

        sleep(5000);

        odometry.update();

        pathSecond.buildPath(whatPath.blueRight);

        follower.setPath(pathSecond.followablePath, pathSecond.pathingVelocity);

        odometry.update();

        follower.followPath(true, 180, false, odometry, drive, telemetry);

//        follower.getPathingVelocity(42, 42);
//
//        horizontal = follower.getPathingVelocity(42, 42).getHorizontal();
//
//        vertical = follower.getPathingVelocity(42, 42).getVertical();
//
//        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);
//
//        double left_Front = (vertical + horizontal + pivot) / denominator;
//        double left_Back = (vertical - horizontal + pivot) / denominator;
//        double right_Front = (vertical - horizontal - pivot) / denominator;
//        double right_Back = (vertical + horizontal - pivot) / denominator;
//
//        drive.RF.setPower(right_Front);
//        drive.RB.setPower(right_Back);
//        drive.LF.setPower(left_Front);
//        drive.LB.setPower(left_Back);
//
//
//        sleep(2000);
//
//        drive.RF.setPower(0);
//        drive.RB.setPower(0);
//        drive.LF.setPower(0);
//        drive.LB.setPower(0);

        while (opModeIsActive()){

//            robotposition.set(244, 120);

//            telemetry.addData("robot pos", robotposition);
//            telemetry.addData("power y", (follower.getPathingPower(robotposition, 180).getHorizontal()));
//            telemetry.addData("power x", (follower.getPathingPower(robotposition, 180).getVertical()));
//            telemetry.addData("x velo", follower.getTargetVelocity(robotposition).getXVelocity());
//            telemetry.addData("y velo", follower.getTargetVelocity(robotposition).getYVelocity());
//            telemetry.update();

            odometry.update();

            telemetry.addData("x opmode", odometry.X);
            telemetry.addData("y opmode", odometry.Y);
            telemetry.addData("heading opmode", odometry.heading);
            telemetry.update();

        }

//        while (opModeIsActive()) {
//
//            counter++;
//
//            odometry.update();
//
//            robotpos.set(odometry.X, odometry.Y);
//
//            if (counter > 50){
//                counter = 0;
//                loopTime = elapsedTime.milliseconds() - lastLoopTime;
//            }
//
//            lastLoopTime = elapsedTime.milliseconds();
//
//            botFullPos.set(odometry.X, odometry.Y, ConvertedHeadingForPosition);
//
////            odometry.update();
////
////            Heading = Odometry.ConvertedHeadingForPosition;
////
////            robotPos.set(odometry.X, odometry.Y);
////
////            //use follower methods to get motor power
////            PathingPower correctivePower;
////            correctivePower = follower.getCorrectivePower(robotPos, Heading);
////
////            Vector2D correctivePosition;
////            correctivePosition = follower.getCorrectivePosition(robotPos);
////
////            PathingPower pathingPower;
////            pathingPower = follower.getPathingPower(robotPos);
////
////            //apply motor power in order of importance
////            if (Math.abs(correctivePosition.getX()) > 5 || Math.abs(correctivePosition.getY()) > 5 && busyPathing) {
////                vertical = correctivePower.getVertical();
////                horizontal = correctivePower.getHorizontal();
////            } else if (!busyPathing) {
////                vertical = correctivePower.getVertical();
////                horizontal = correctivePower.getHorizontal();
////            } else {
////                horizontal = pathingPower.getHorizontal();
////                vertical = pathingPower.getVertical();
////            }
////
////            if (pathingPower.getHorizontal() < 0.08 && pathingPower.getVertical() < 0.08){
////                busyPathing = false;
////            }else {
////                busyPathing = true;
////            }
////
////            pivot = follower.getTurnPower(0, Heading);
////
////            //apply motor powers
////            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);
////
////            double left_Front = (vertical + horizontal + pivot) / denominator;
////            double left_Back = (vertical - horizontal + pivot) / denominator;
////            double right_Front = (vertical - horizontal - pivot) / denominator;
////            double right_Back = (vertical + horizontal - pivot) / denominator;
////
////            drive.RF.setPower(right_Front);
////            drive.RB.setPower(right_Back);
////            drive.LF.setPower(left_Front);
////            drive.LB.setPower(left_Back);
//
//            telemetry.addData("loop time ", loopTime);
//            telemetry.addData("Power X", vertical);
//            telemetry.addData("Power Y", horizontal);
//            telemetry.addData("turn power", pivot);
//            telemetry.addData("Robot pos", robotpos);
//            telemetry.update();
//        }

    }
}
