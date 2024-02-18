package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.teleopPathBuilder;

@TeleOp
@Config
@Disabled
public class TestingSnap extends OpMode implements TeleopPathing{

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    teleopPathBuilder path = new teleopPathBuilder();

    Vector2D robotPos = new Vector2D();

    mecanumFollower follower = new mecanumFollower();

    boolean pathing = false;

    boolean headingLock = false;

    boolean snapToBackboard = false;

    boolean inBackboardArea = false;

    boolean firstSnap = false;

    public static int snapPos;

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        snapToBackboard = true;

        if (snapPos > 7){
            snapPos = 1;
        }

        if (snapPos < 1){
            snapPos = 7;
        }

        if(snapPos == 1){

            double xerror = Math.abs(threeLeftRed.getX() - robotPos.getX());
            double yerror = Math.abs(threeLeftRed.getY() - robotPos.getY());

            if (xerror < 2 && yerror < 2){

            }else {
                path.buildPathLine(robotPos, threeLeftRed);
            }


        }else if(snapPos == 2){

            double xerror = Math.abs(twoLeftRed.getX() - robotPos.getX());
            double yerror = Math.abs(twoLeftRed.getY() - robotPos.getY());

            if (xerror < 2 && yerror < 2){

            }else {
                path.buildPathLine(robotPos, twoLeftRed);
            }

        }else if(snapPos == 3){

            double xerror = Math.abs(oneLeftRed.getX() - robotPos.getX());
            double yerror = Math.abs(oneLeftRed.getY() - robotPos.getY());

            if (xerror < 2 && yerror < 2){

            }else {
                path.buildPathLine(robotPos, oneLeftRed);
            }

        }else if(snapPos == 4){

            double xerror = Math.abs(middleRed.getX() - robotPos.getX());
            double yerror = Math.abs(middleRed.getY() - robotPos.getY());

            if (xerror < 2 && yerror < 2){

            }else {
                path.buildPathLine(robotPos, middleRed);
            }


        }else if(snapPos == 5){

            double xerror = Math.abs(oneRightRed.getX() - robotPos.getX());
            double yerror = Math.abs(oneRightRed.getY() - robotPos.getY());

            if (xerror < 2 && yerror < 2){

            }else {
                path.buildPathLine(robotPos, oneRightRed);
            }

        }else if(snapPos == 6){

            double xerror = Math.abs(twoRightRed.getX() - robotPos.getX());
            double yerror = Math.abs(twoRightRed.getY() - robotPos.getY());

            if (xerror < 2 && yerror < 2){

            }else {
                path.buildPathLine(robotPos, twoRightRed);
            }

        }else if(snapPos == 7){

            double xerror = Math.abs(threeRightRed.getX() - robotPos.getX());
            double yerror = Math.abs(threeRightRed.getY() - robotPos.getY());

            if (xerror < 2 && yerror < 2){

            }else {
                path.buildPathLine(robotPos, threeRightRed);
            }

        }

        follower.setPath(path.followablePath, path.pathingVelocity);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("snapPos", snapPos);
        dashboard.sendTelemetryPacket(packet);

        dashboardTelemetry.addData("path lengh", path.followablePath.size());
        dashboardTelemetry.addData("path lengh follower", follower.getPathLength()/0.25);
        dashboardTelemetry.update();

    }
}
