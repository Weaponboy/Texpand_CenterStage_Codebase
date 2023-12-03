package org.firstinspires.ftc.teamcode.hardware.Odometry.Pathing.Testing;

import static org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.KDTreeExample.findNearestNeighbor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.KDTreeExample;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.ObstacleMap;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.buildRobotBoundary;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class TestingObjectAvoidanceNoRobot extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    buildRobotBoundary buildrobotboundary = new buildRobotBoundary();

    Vector2D robotPos = new Vector2D();

    public static double X = 30;
    public static double Y = 30;

    int counter;

    ElapsedTime elapsedTime = new ElapsedTime();

    ObstacleMap obstacleMap;

    double lastLoopTime;

    double loopTime;

    KDTreeExample kdTreeExample = new KDTreeExample();

    @Override
    public void init() {
        dashboard.setTelemetryTransmissionInterval(25);
        obstacleMap = new ObstacleMap();
    }

    @Override
    public void loop() {

        counter++;

        if (counter > 50){
            counter = 0;
            loopTime = elapsedTime.milliseconds() - lastLoopTime;
        }

        lastLoopTime = elapsedTime.milliseconds();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("X", X);
        packet.put("Y", Y);
        dashboard.sendTelemetryPacket(packet);

        robotPos.set(X, Y);

        Vector2D closestPos = kdTreeExample.buildRobot(robotPos, obstacleMap.positionList);

        dashboardTelemetry.addData("position", kdTreeExample.buildRobot(robotPos, obstacleMap.positionList));
        dashboardTelemetry.addData("loop time", loopTime);
        dashboardTelemetry.update();

    }



}
