package org.firstinspires.ftc.teamcode.Auto.OtherAuto.Test_Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;

@TeleOp
public class TestingNewPathFollower extends LinearOpMode {

    Odometry odometry = new Odometry(90, 23, 270);

    blueRightBuilder preloadPurple = new blueRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void runOpMode() throws InterruptedException {
        odometry.init(hardwareMap);

        waitForStart();

        preloadPurple.buildPath(blueRightBuilder.Position.center, blueRightBuilder.Section.preload);

        follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);

        follower.TestNewPathingMethod(odometry);

    }

}
