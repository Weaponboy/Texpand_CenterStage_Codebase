package org.firstinspires.ftc.teamcode.Auto.OtherAuto.Test_Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;

@TeleOp
public class Testing_loop_times extends LinearOpMode {

    blueRightBuilder blueRightBuilder = new blueRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Vector2D robotPos = new Vector2D(0,0);

    @Override
    public void runOpMode() throws InterruptedException {

        blueRightBuilder.buildPath(org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder.Position.center, org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder.Section.preload);

        follower.setPath(blueRightBuilder.followablePath, blueRightBuilder.pathingVelocity);

        waitForStart();

        follower.testingLoopTime(180, robotPos, 180);

    }
}
