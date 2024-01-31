package org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.ObstacleMapGVF;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.ObstacleMap;

public class TestingVectorGen extends LinearOpMode {

    ObstacleMapGVF obstacleMapGVF = new ObstacleMapGVF();

    @Override
    public void runOpMode() throws InterruptedException {
        obstacleMapGVF.SetMap();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("point", obstacleMapGVF.realObstacle.get(200));
            telemetry.addData("velocity", obstacleMapGVF.VectorPowers.get(200));
            telemetry.update();
        }
    }

}
