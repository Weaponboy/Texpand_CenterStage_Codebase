package org.firstinspires.ftc.teamcode.hardware.Odometry.Pathing.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.ObstacleMap;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Odometry;

@Config
@TeleOp
public class testObstacleAvoidance extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ElapsedTime elapsedTime = new ElapsedTime();

    Odometry odometry = new Odometry(210, 337, 90);

    Vector2D robotPos = new Vector2D();

    ObstacleMap obstacleMap = new ObstacleMap();

    int counter;

    public static double X = 0;
    public static double Y = 0;

    double lastLoopTime;

    double loopTime;

    @Override
    public void init() {

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

        obstacleMap.buildRobotPosition(robotPos, 0);

//        double xPower = horizontal * Math.sin(Math.toRadians(ConvertedHeadingForPosition)) + vertical * Math.cos(Math.toRadians(ConvertedHeadingForPosition));
//        double yPower = horizontal * Math.cos(Math.toRadians(ConvertedHeadingForPosition)) - vertical * Math.sin(Math.toRadians(ConvertedHeadingForPosition));

//        double denominator = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(pivot), 1);

//        drive.RF.setPower((-pivot + (xPower - yPower)) / denominator);
//        drive.RB.setPower((-pivot + (xPower + yPower)) / denominator);
//        drive.LF.setPower((pivot + (xPower + yPower)) / denominator);
//        drive.LB.setPower((pivot + (xPower - yPower)) / denominator);

        telemetry.addData("closestPos", obstacleMap.robotPosition.get(10));
        telemetry.addData("robot pos", robotPos);
        telemetry.update();

    }


}





