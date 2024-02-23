package org.firstinspires.ftc.teamcode.Odometry.Pathing.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Enums.whatPath;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Old.pathBuilder;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;

@TeleOp
@Disabled
public class LineTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    pathBuilder pathFirst = new pathBuilder();

    Odometry odometry = new Odometry(0,0,0);

    Drivetrain drive = new Drivetrain();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        //build path
        pathFirst.buildPath(whatPath.blueRight);

        follower.setPath(pathFirst.followablePath, pathFirst.pathingVelocity);

        odometry.update();

        waitForStart();

        while (!(Math.abs(odometry.X - 80) < 2) || !(Math.abs(odometry.Y - 0) < 2)){
            follower.followPathAuto(0, odometry, drive);
        }

    }
}
