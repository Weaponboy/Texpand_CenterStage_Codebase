package org.firstinspires.ftc.teamcode.Odometry.Pathing.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.robotPos;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Old.pathBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Enums.whatPath;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

@TeleOp
public class TestingNewGenWithoutRobot extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    pathBuilder pathFirst = new pathBuilder();

    pathBuilder pathSecond = new pathBuilder();

    Odometry odometry = new Odometry(93,23, 270);

    Drivetrain drive = new Drivetrain();

    ElapsedTime elapsedTime = new ElapsedTime();

    robotPos botFullPos = new robotPos();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        odometry.init(hardwareMap);
        drive.init(hardwareMap);

        //build path
        pathFirst.buildPath(whatPath.testCurve);

        follower.setPath(pathFirst.followablePath, pathFirst.pathingVelocity);

        waitForStart();

        odometry.update(0);

        follower.followPath(false, 180, false, odometry, drive, telemetry);

        odometry.update(0);

//        pathSecond.buildPath(whatPath.blueRight);
//
//        follower.setPath(pathSecond.followablePath, pathSecond.pathingVelocity);
//
//        botFullPos.set(X, Y, Heading);
//
//        follower.followPath(botFullPos, false, 180, true, odometry);

        while (opModeIsActive()){

            dashboardTelemetry.addData("x opmode", odometry.X);
            dashboardTelemetry.addData("y opmode", odometry.Y);
            dashboardTelemetry.addData("heading opmode", odometry.heading);
            dashboardTelemetry.update();

        }

    }
}
