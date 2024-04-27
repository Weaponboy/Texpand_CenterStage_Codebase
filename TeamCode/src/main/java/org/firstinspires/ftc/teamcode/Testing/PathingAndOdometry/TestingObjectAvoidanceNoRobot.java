package org.firstinspires.ftc.teamcode.Testing.PathingAndOdometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.KDTreeExample;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.ObstacleMap;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.buildRobotBoundary;

import java.util.List;

@Disabled
@TeleOp
@Config
public class TestingObjectAvoidanceNoRobot extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    buildRobotBoundary buildrobotboundary = new buildRobotBoundary();

    Vector2D robotPos = new Vector2D();

    public static double X = 30;
    public static double Y = 30;
    public static double Heading = 0;

    public static double inputVertical = 0;
    public static double inputHorizontal = 0;

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
        packet.put("Heading", Heading);
        packet.put("vertical input", inputVertical);
        packet.put("horizontal input", inputHorizontal);
        dashboard.sendTelemetryPacket(packet);

        robotPos.set(X, Y);

        buildrobotboundary.buildRobotPosition(robotPos, Heading);

        double outputVertical = getXInLine(kdTreeExample.findClosestPoint(robotPos, obstacleMap.realObstacles), buildrobotboundary.robotPosition, inputVertical, inputHorizontal, Heading);
        double outputHorizontal = getYInLine(kdTreeExample.findClosestPoint(robotPos, obstacleMap.realObstacles), buildrobotboundary.robotPosition, inputVertical, inputHorizontal, Heading);

        dashboardTelemetry.addData("output vertical", outputVertical);
        dashboardTelemetry.addData("output horizontal", outputHorizontal);
        dashboardTelemetry.addData("loop time", loopTime);
        dashboardTelemetry.update();

    }

    public double getXInLine(Vector2D closestObstacleToRobotPosition, List<Vector2D> robotPosition, double inputVertical, double inputHorizontal, double heading){

        Vector2D returnPosition = kdTreeExample.findInlineNeighborX(robotPosition, closestObstacleToRobotPosition, dashboardTelemetry);

        double vertical = inputVertical * Math.cos(Math.toRadians(360 - heading)) - inputHorizontal * Math.sin(Math.toRadians(360 - heading));

        double horizontal = inputVertical * Math.sin(Math.toRadians(360 - heading)) + inputHorizontal * Math.cos(Math.toRadians(360 - heading));

        if (returnPosition == null){

        }else{
            if (closestObstacleToRobotPosition.getX() - returnPosition.getX() < 5 && closestObstacleToRobotPosition.getX() - returnPosition.getX() > 0){
                vertical = 0;
            } else if (closestObstacleToRobotPosition.getX() - returnPosition.getX() > -5 && closestObstacleToRobotPosition.getX() - returnPosition.getX() < 0) {
                vertical = 0;
            }

            if (closestObstacleToRobotPosition.getX() - returnPosition.getX() < 10 && closestObstacleToRobotPosition.getX() - returnPosition.getX() > 0){
                vertical = 0;
            } else if (closestObstacleToRobotPosition.getX() - returnPosition.getX() > -10 && closestObstacleToRobotPosition.getX() - returnPosition.getX() < 0) {
                vertical = 0;
            }
        }

        return horizontal * Math.sin(Math.toRadians(360 - heading)) + vertical * Math.cos(Math.toRadians(360 - heading));
    }

    public double getYInLine(Vector2D closestObstacleToRobotPosition, List<Vector2D> robotPosition, double inputVertical, double inputHorizontal, double heading){

        Vector2D returnPosition = kdTreeExample.findInlineNeighborY(robotPosition, closestObstacleToRobotPosition);

        double vertical = inputVertical * Math.cos(Math.toRadians(360 - heading)) - inputHorizontal * Math.sin(Math.toRadians(360 - heading));
        double horizontal = inputVertical * Math.sin(Math.toRadians(360 - heading)) + inputHorizontal * Math.cos(Math.toRadians(360 - heading));

        if (returnPosition == null){

        }else{
            if (closestObstacleToRobotPosition.getY() - returnPosition.getY() < 5 && closestObstacleToRobotPosition.getY() - returnPosition.getY() > 0){
                horizontal = 0;
            } else if (closestObstacleToRobotPosition.getY() - returnPosition.getY() > -5 && closestObstacleToRobotPosition.getY() - returnPosition.getY() < 0) {
                horizontal = 0;
            }

            if (closestObstacleToRobotPosition.getY() - returnPosition.getY() < 10 && closestObstacleToRobotPosition.getY() - returnPosition.getY() > 0){
                horizontal = 0;
            } else if (closestObstacleToRobotPosition.getY() - returnPosition.getY() > -10 && closestObstacleToRobotPosition.getY() - returnPosition.getY() < 0) {
                horizontal = 0;
            }

        }


        return horizontal * Math.cos(Math.toRadians(360 - heading)) - vertical * Math.sin(Math.toRadians(360 - heading));
    }

}
