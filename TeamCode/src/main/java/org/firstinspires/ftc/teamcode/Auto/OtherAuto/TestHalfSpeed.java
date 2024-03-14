package org.firstinspires.ftc.teamcode.Auto.OtherAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.GenMethods;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;

public class TestHalfSpeed extends LinearOpMode {

    GenMethods preloadPaths = new GenMethods();

    Odometry odometry = new Odometry(0, 0, 0);

    Drivetrain drive = new Drivetrain();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void runOpMode() throws InterruptedException {

        preloadPaths.buildLineSegment(new Vector2D(0,0), new Vector2D(120, 0));

        follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);

        waitForStart();

        while (opModeIsActive()){
            follower.followPathAuto(0, odometry, drive);
        }

    }

}
