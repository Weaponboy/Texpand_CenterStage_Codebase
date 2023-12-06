package org.firstinspires.ftc.teamcode.hardware.Odometry.Pathing.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.KDTreeExample;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.ObstacleMap;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.buildRobotBoundary;
import org.firstinspires.ftc.teamcode.hardware.Odometry.Pathing.PathingPower.PathingPower;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Odometry;

import java.util.List;

@Config
@TeleOp
public class testObstacleAvoidance extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ElapsedTime elapsedTime = new ElapsedTime();

    Odometry odometry = new Odometry(210, 337, 90);

    Drivetrain drive = new Drivetrain();

    Vector2D robotPos = new Vector2D();

    ObstacleMap obstacleMap = new ObstacleMap();

    int counter;

    public static double X = 0;
    public static double Y = 0;

    double lastLoopTime;

    double loopTime;

    buildRobotBoundary buildrobotboundary = new buildRobotBoundary();

    KDTreeExample kdTreeExample = new KDTreeExample();

    @Override
    public void init() {

        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        counter++;

        if (counter > 50){
            counter = 0;
            loopTime = elapsedTime.milliseconds() - lastLoopTime;
        }

        lastLoopTime = elapsedTime.milliseconds();

        robotPos.set(X, Y);

        counter++;

        if (counter > 50){
            counter = 0;
            loopTime = elapsedTime.milliseconds() - lastLoopTime;
        }

        lastLoopTime = elapsedTime.milliseconds();

        odometry.update();

        robotPos.set(odometry.X, odometry.Y);

        buildrobotboundary.buildRobotPosition(robotPos, odometry.heading);

        double inputVertical = -gamepad1.right_stick_y;
        double inputHorizontal = gamepad1.right_stick_x;
        double inputRotation = gamepad1.left_stick_x;

        double outputVertical = getXInLine(kdTreeExample.findClosestPoint(robotPos, obstacleMap.positionList), buildrobotboundary.robotPosition, inputVertical, inputHorizontal, odometry.heading);
        double outputHorizontal = getYInLine(kdTreeExample.findClosestPoint(robotPos, obstacleMap.positionList), buildrobotboundary.robotPosition, inputVertical, inputHorizontal, odometry.heading);

        drive.RF.setPower((-inputRotation + (outputVertical - outputHorizontal)));
        drive.RB.setPower((-inputRotation + (outputVertical + outputHorizontal)));
        drive.LF.setPower((inputRotation + (outputVertical + outputHorizontal)));
        drive.LB.setPower((inputRotation + (outputVertical - outputHorizontal)));

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

            if (closestObstacleToRobotPosition.getX() - returnPosition.getX() < 10 && closestObstacleToRobotPosition.getX() - returnPosition.getX() > 0 && odometry.getVerticalVelocity() > 20){
                vertical = 0;
            } else if (closestObstacleToRobotPosition.getX() - returnPosition.getX() > -10 && closestObstacleToRobotPosition.getX() - returnPosition.getX() < 0 && odometry.getVerticalVelocity() < -20) {
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

            if (closestObstacleToRobotPosition.getY() - returnPosition.getY() < 10 && closestObstacleToRobotPosition.getY() - returnPosition.getY() > 0 && odometry.getHorizontalVelocity() > 20){
                horizontal = 0;
            } else if (closestObstacleToRobotPosition.getY() - returnPosition.getY() > -10 && closestObstacleToRobotPosition.getY() - returnPosition.getY() < 0 && odometry.getHorizontalVelocity() < -20) {
                horizontal = 0;
            }

        }


        return horizontal * Math.cos(Math.toRadians(360 - heading)) - vertical * Math.sin(Math.toRadians(360 - heading));
    }

}





