package org.firstinspires.ftc.teamcode.Testing.PathingAndOdometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Old.pathBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Enums.whatPath;

@TeleOp
@Disabled
public class TestingNewGenWithoutRobot extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    pathBuilder pathFirst = new pathBuilder();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //build path
        pathFirst.buildPath(whatPath.blueRight);

        follower.setPath(pathFirst.followablePath, pathFirst.pathingVelocity);

        waitForStart();


        while (opModeIsActive()){

            dashboardTelemetry.addData("x opmode", pathFirst.followablePath.get(200));
            dashboardTelemetry.addData("x velo", pathFirst.pathingVelocity.get(200).getXVelocity());
            dashboardTelemetry.addData("y velo", pathFirst.pathingVelocity.get(200).getYVelocity());
            dashboardTelemetry.update();

        }

    }
}
