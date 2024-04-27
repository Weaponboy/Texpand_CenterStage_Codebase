package org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.vertical;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingPower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

import java.util.ArrayList;

@Config
public class mecanumFollower {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    double yI = 0;
    double xI = 0;

    public static double X = 0;
    public static double Y = 0;
    boolean gotToEnd = false;

    double loopTime;
    int counter;
    double lastLoopTime;

    ElapsedTime elapsedTime = new ElapsedTime();

    ElapsedTime reverse = new ElapsedTime();
    ElapsedTime getCorrective = new ElapsedTime();
    double time = 0;

    HardwareMap hardwareMap;

    PIDController XCorrective = new PIDController(driveP, xI, driveD);
    PIDController YCorrective = new PIDController(strafeP, xI, strafeD);
    PIDController headingPID;

    FollowPath pathfollow;

    public void setPath(ArrayList<Vector2D> trajectory, ArrayList<PathingVelocity> pathingVelocity){
        pathfollow = new FollowPath(trajectory, pathingVelocity);
    }

    public void resetClosestPoint(Vector2D robotPos){
        pathfollow.getClosestPositionOnPathFullPath(robotPos);
    }

    public PathingPower getPathingPower(Vector2D robotPos, double heading){

        PathingPower pathingPower;
        PathingPower actualPathingPower = new PathingPower();

        double ky = 0.0234;
        double kx = 0.0154;

        PathingVelocity pathingVelocity;

        getCorrective.reset();

        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);

        time = getCorrective.milliseconds();

        pathingVelocity = pathfollow.getTargetVelocity(closestPos);//try this

        double xPower = pathingVelocity.getYVelocity() * Math.sin(Math.toRadians(heading)) + pathingVelocity.getXVelocity() * Math.cos(Math.toRadians(heading));
        double yPower = pathingVelocity.getYVelocity() * Math.cos(Math.toRadians(heading)) - pathingVelocity.getXVelocity() * Math.sin(Math.toRadians(heading));

        vertical = kx * xPower;
        horizontal = ky * yPower;

        pathingPower = new PathingPower(vertical, horizontal);

        return pathingPower;
    }

    public PathingPower getFullPathingPower(Vector2D robotPos, double heading, Odometry odometry) {

        XCorrective.setPID(driveP, 0, driveD);
        YCorrective.setPID(strafeP, 0, strafeD);

        Vector2D error;

        PathingPower actualPathingPower = new PathingPower();

        double kyfull = 0.0079;
        double kxfull = 0.0053;

        double ky = 0.0053;
        double kx = 0.00346;

        PathingVelocity targetPathingVelocity;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);

        targetPathingVelocity = pathfollow.getTargetVelocity(closestPos);

        double robotXVelocity = odometry.getVerticalVelocity();
        double robotYVelocity = odometry.getHorizontalVelocity();

        double currentXVelo = robotXVelocity * Math.cos(Math.toRadians(heading)) - robotYVelocity * Math.sin(Math.toRadians(heading));
        double currentYVelo = robotXVelocity * Math.sin(Math.toRadians(heading)) + robotYVelocity * Math.cos(Math.toRadians(heading));

        double adjustedX = 0;
        double adjustedY = 0;

        if (targetPathingVelocity.getXVelocity() > 0) {

            if (currentXVelo > targetPathingVelocity.getXVelocity()) {

                adjustedX = targetPathingVelocity.getXVelocity() - (currentXVelo - targetPathingVelocity.getXVelocity());

            } else if (currentXVelo < targetPathingVelocity.getXVelocity()) {

                adjustedX = targetPathingVelocity.getXVelocity() + (targetPathingVelocity.getXVelocity() - currentXVelo);

            }

        } else if (targetPathingVelocity.getXVelocity() < 0) {

            if (currentXVelo > targetPathingVelocity.getXVelocity()) {

                adjustedX = targetPathingVelocity.getXVelocity() + (Math.abs(currentXVelo) - Math.abs(targetPathingVelocity.getXVelocity()));

            } else if (currentXVelo < targetPathingVelocity.getXVelocity()) {

                adjustedX = targetPathingVelocity.getXVelocity() - (Math.abs(targetPathingVelocity.getXVelocity()) - Math.abs(currentXVelo));

            }

        }

        if (targetPathingVelocity.getYVelocity() > 0) {

            if (currentYVelo > targetPathingVelocity.getYVelocity()) {

                adjustedY = targetPathingVelocity.getYVelocity() - (currentYVelo - targetPathingVelocity.getYVelocity());

            } else if (currentYVelo < targetPathingVelocity.getYVelocity()) {

                adjustedY = targetPathingVelocity.getYVelocity() + (targetPathingVelocity.getYVelocity() - currentYVelo);

            }

        } else if (targetPathingVelocity.getYVelocity() < 0) {

            if (currentYVelo > targetPathingVelocity.getYVelocity()) {

                adjustedY = targetPathingVelocity.getYVelocity() + (Math.abs(currentYVelo) - Math.abs(targetPathingVelocity.getYVelocity()));

            } else if (currentYVelo < targetPathingVelocity.getYVelocity()) {

                adjustedY = targetPathingVelocity.getYVelocity() - (Math.abs(targetPathingVelocity.getYVelocity()) - Math.abs(currentYVelo));

            }

        }

        error = pathfollow.getErrorToPath(robotPos, closestPos);

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPowerC = XCorrective.calculate(-robotRelativeXError);
        double yPowerC = YCorrective.calculate(-robotRelativeYError) * 1.5;

        double xPower = adjustedY * Math.sin(Math.toRadians(heading)) + adjustedX * Math.cos(Math.toRadians(heading));
        double yPower = adjustedY * Math.cos(Math.toRadians(heading)) - adjustedX * Math.sin(Math.toRadians(heading));

        vertical = kxfull * xPower;
        horizontal = kyfull * yPower;

        if (horizontal > 1) {
            vertical = kx * xPower;
            horizontal = ky * yPower;
        }

        double vertical2 = vertical;
        double horizontal2 = horizontal;

        if (closestPos > 100) {

            if (Math.abs(xPower - odometry.getVerticalVelocity()) > 100 && Math.abs(yPower - odometry.getHorizontalVelocity()) > 80 && reverse.milliseconds() > 300) {

                System.out.println("vertical " + Math.abs(xPower - odometry.getVerticalVelocity()));
                System.out.println("horizontal " + Math.abs(yPower - odometry.getHorizontalVelocity()));

                reverse.reset();
            }

            if (reverse.milliseconds() < 200) {
//
//                vertical2 = -vertical;
//                horizontal2 = -horizontal;

            } else if (reverse.milliseconds() > 200 && reverse.milliseconds() < 300) {

//                vertical = 0;
//                horizontal = 0;

            }

        }

        actualPathingPower.set(xPowerC + vertical2, yPowerC + horizontal2);

        return actualPathingPower;
    }

    public PathingPower getFullPathingPower(Vector2D robotPos, double heading, Telemetry telemetry){

        XCorrective.setPID(driveP, 0, driveD);
        YCorrective.setPID(strafeP, 0, strafeD);

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        PathingPower pathingPower;
        PathingPower actualPathingPower = new PathingPower();

        double kyfull = 0.0234;
        double kxfull = 0.0154;

        double ky = 0.0154;
        double kx = 0.01;

        PathingVelocity pathingVelocity;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);

        double curveY = 0.7278975375;
        double curveX = 0.7278975375;

        int index = closestPos + 5;

        if (index > pathfollow.pathCurve.size()-1){
            index = pathfollow.pathCurve.size()-1;
        }

        double XpowerCurve;
        double YpowerCurve;

        if (index > 400){
            XpowerCurve = pathfollow.pathCurve.get(index).getX()*curveX;
            YpowerCurve = pathfollow.pathCurve.get(index).getY()*curveY;
        }else {
            XpowerCurve = 0;
            YpowerCurve = 0;
        }

        double robotRelativeXCurve = YpowerCurve * Math.sin(Math.toRadians(heading)) + XpowerCurve * Math.cos(Math.toRadians(heading));
        double robotRelativeYCurve = YpowerCurve * Math.cos(Math.toRadians(heading)) - XpowerCurve * Math.sin(Math.toRadians(heading));

        pathingVelocity = pathfollow.getTargetVelocity(closestPos);

        telemetry.addData("x power", pathingVelocity.getXVelocity());
        telemetry.addData("y power", pathingVelocity.getYVelocity());
        telemetry.addData("x position", getPointOnPath(closestPos).getX());
        telemetry.addData("y position", getPointOnPath(closestPos).getY());

        error = pathfollow.getErrorToPath(robotPos, closestPos);

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPowerC = XCorrective.calculate(-robotRelativeXError);
        double yPowerC = YCorrective.calculate(-robotRelativeYError)*1.5;

        double xPower = pathingVelocity.getYVelocity() * Math.sin(Math.toRadians(heading)) + pathingVelocity.getXVelocity() * Math.cos(Math.toRadians(heading));
        double yPower = pathingVelocity.getYVelocity() * Math.cos(Math.toRadians(heading)) - pathingVelocity.getXVelocity() * Math.sin(Math.toRadians(heading));

        vertical = kxfull * xPower;
        horizontal = kyfull * yPower;

        if(horizontal > 1){
            vertical = kx * xPower;
            horizontal = ky * yPower;
        }

        actualPathingPower.set(xPowerC+vertical+robotRelativeXCurve, yPowerC+horizontal+robotRelativeYCurve);

        return actualPathingPower;
    }

//    public PathingPower getFullPathingPower(Vector2D robotPos, double heading){
//
//        XCorrective.setPID(driveP, 0, driveD);
//        YCorrective.setPID(strafeP, 0, strafeD);
//
//        Vector2D error;
//        PathingPower actualPathingPower = new PathingPower();
//
//        double kyfull = 0.0349;
//        double kxfull = 0.0234;
//
//        double ky = 0.0234;
//        double kx = 0.0154;
//
//        PathingVelocity pathingVelocity;
//
//        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);
//
//        pathingVelocity = pathfollow.getTargetVelocity(closestPos);
//
//        double curveY = 0.7278975375;
//        double curveX = 0.7278975375;
//
//        int index = closestPos + 5;
//
//        if (index > pathfollow.pathCurve.size()-1){
//            index = pathfollow.pathCurve.size()-1;
//        }
//
//        double XpowerCurve;
//        double YpowerCurve;
//
//        if (index > 200){
//            XpowerCurve = pathfollow.pathCurve.get(index).getX()*curveX;
//            YpowerCurve = pathfollow.pathCurve.get(index).getY()*curveY;
//        }else {
//            XpowerCurve = 0;
//            YpowerCurve = 0;
//        }
//
//        double robotRelativeXCurve = YpowerCurve * Math.sin(Math.toRadians(heading)) + XpowerCurve * Math.cos(Math.toRadians(heading));
//        double robotRelativeYCurve = YpowerCurve * Math.cos(Math.toRadians(heading)) - XpowerCurve * Math.sin(Math.toRadians(heading));
//
//        error = pathfollow.getErrorToPath(robotPos, closestPos);
//
//        double xDist = error.getX();
//        double yDist = error.getY();
//
//        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
//        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));
//
//        double xPowerC = XCorrective.calculate(robotRelativeXError);
//        double yPowerC = YCorrective.calculate(robotRelativeYError);
//
//        double xPower = pathingVelocity.getYVelocity() * Math.sin(Math.toRadians(heading)) + pathingVelocity.getXVelocity() * Math.cos(Math.toRadians(heading));
//        double yPower = pathingVelocity.getYVelocity() * Math.cos(Math.toRadians(heading)) - pathingVelocity.getXVelocity() * Math.sin(Math.toRadians(heading));
//
////        System.out.println(pathingVelocity.getYVelocity());
////        System.out.println(pathingVelocity.getXVelocity());
//
//        double vertical = kxfull * xPower;
//        double horizontal = kyfull * yPower;
//
//        if(horizontal > 1){
//            vertical = kx * xPower;
//            horizontal = ky * yPower;
//        }
//
//        actualPathingPower.set(vertical+xPowerC+robotRelativeXCurve, horizontal+yPowerC+robotRelativeYCurve);
//
//        return actualPathingPower;
//    }

    public PathingPower getCorrectivePowerAtEnd(Vector2D robotPos, Vector2D targetPos, double heading){

        XCorrective.setPID(0.045, xI, 0.0001);
        YCorrective.setPID(0.045, yI, 0.0001);

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        error = new Vector2D( targetPos.getX() - robotPos.getX(),  targetPos.getY() - robotPos.getY());

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPower = XCorrective.calculate(-robotRelativeXError);
        double yPower = YCorrective.calculate(-robotRelativeYError)*1.5;

        correctivePower.set(xPower, yPower);

        return correctivePower;
    }

    public PathingPower getPathingPowerInverted(Vector2D robotPos, double heading){

        PathingPower pathingPower;
        PathingPower actualPathingPower = new PathingPower();

        double ky = 0.0234;
        double kx = 0.0154;

        PathingVelocity pathingVelocity;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);

        pathingVelocity = pathfollow.getTargetVelocity(closestPos);//try this

        double xPower = pathingVelocity.getYVelocity() * Math.sin(Math.toRadians(heading)) + pathingVelocity.getXVelocity() * Math.cos(Math.toRadians(heading));
        double yPower = pathingVelocity.getYVelocity() * Math.cos(Math.toRadians(heading)) - pathingVelocity.getXVelocity() * Math.sin(Math.toRadians(heading));

        vertical = -(kx * xPower);
        horizontal = -(ky * yPower);

        pathingPower = new PathingPower(vertical, horizontal);

        return pathingPower;
    }

    public PathingPower getPathingVelocity(double xVelo, double yVelo){

        PathingPower pathingPower;

        double ky = 0.0235;
        double kx = 0.0154;

        double xPower = yVelo * Math.sin(Math.toRadians(0)) + xVelo * Math.cos(Math.toRadians(0));
        double yPower = yVelo * Math.cos(Math.toRadians(0)) - xVelo * Math.sin(Math.toRadians(0));

        vertical = kx * xPower;
        horizontal = ky * yPower;

        pathingPower = new PathingPower(vertical, horizontal);

        return pathingPower;
    }

    public PathingPower getCorrectivePowerOnPath(Vector2D robotPos, double heading){

        XCorrective.setPID(driveP, 0, driveD);
        YCorrective.setPID(strafeP, 0, strafeD);

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        error = pathfollow.getErrorToPath(robotPos);

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPower = XCorrective.calculate(-robotRelativeXError)*1.2;
        double yPower = YCorrective.calculate(-robotRelativeYError)*1.4;

        correctivePower.set(xPower, yPower);

        return correctivePower;
    }

    public Vector2D getCorrectivePosition(Vector2D robotPos){

        Vector2D error;

        error = pathfollow.getErrorToPath(robotPos);

        return error;
    }

    public PathingVelocity getTargetVelocity(Vector2D robotPos){
        PathingVelocity pathingVelocity;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);

        return pathingVelocity = pathfollow.getTargetVelocity(closestPos);
    }

    public boolean followPathTeleop(double targetHeading, Odometry odometry, Drivetrain drive){

        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean pathing = true;

        robotPositionVector.set(odometry.X, odometry.Y);

        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
            pathing = false;
        }

        PathingPower correctivePower;
        PathingPower pathingPower;

        if(Math.abs(odometry.getHorizontalVelocity()) < 3){
            yI += 0.01;
        }else {
            yI = 0;
        }

        if(Math.abs(odometry.getVerticalVelocity()) < 3){
            xI += 0.008;
        }else {
            xI = 0;
        }

        double heading = odometry.heading;

        pathingPower = getPathingPower(robotPositionVector, heading);
        correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);

        vertical = correctivePower.getVertical() + pathingPower.getVertical();
        horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();

        pivot = getTurnPower(targetHeading, odometry.heading);

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        double left_Front = (vertical + horizontal + pivot) / denominator;
        double left_Back = (vertical - horizontal + pivot) / denominator;
        double right_Front = (vertical - horizontal - pivot) / denominator;
        double right_Back = (vertical + horizontal - pivot) / denominator;

        drive.RF.setPower(right_Front);
        drive.RB.setPower(right_Back);
        drive.LF.setPower(left_Front);
        drive.LB.setPower(left_Back);

        return pathing;

    }

//    public boolean followPathAuto(double targetHeading, Odometry odometry, Drivetrain drive){
//
//        odometry.update();
//
//        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);
//
//        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());
//
//        boolean pathing = true;
//
//        robotPositionVector.set(odometry.X, odometry.Y);
//
//        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
//            pathing = false;
//        }
//
//        boolean closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 10 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 10;
//
//        PathingPower correctivePower = new PathingPower();
//        PathingPower pathingPower;
//
//        if(Math.abs(odometry.getHorizontalVelocity()) < 3){
//            yI += 0.05;
//        }else {
//            yI = 0;
//        }
//
//        if(Math.abs(odometry.getVerticalVelocity()) < 3){
//            xI += 0.05;
//        }else {
//            xI = 0;
//        }
//
//        double heading = odometry.heading;
//
//        int closestPos = pathfollow.getClosestPositionOnPath(robotPositionVector);
//
//        if (pathfollow.followablePath.size()-1 > 700){
//            if (closestPos >= pathfollow.followablePath.size()-200){
//                gotToEnd = true;
//            } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
//                gotToEnd = false;
//            }
//        }else {
//            if (closestPos >= pathfollow.followablePath.size()-150){
//                gotToEnd = true;
//            } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
//                gotToEnd = false;
//            }
//        }
//
//        RobotLog.d("X" + odometry.X);
//        RobotLog.d("Y" + odometry.Y);
//        RobotLog.d("heading" + odometry.heading);
//
//        if (!gotToEnd){
//            pathingPower = getFullPathingPower(robotPositionVector, heading);
//            System.out.println(closestPos);
//            System.out.println(pathfollow.followablePath.size()-1);
//        }else {
//            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
//            pathingPower = new PathingPower(0,0);
//        }
//
//        vertical = correctivePower.getVertical() + pathingPower.getVertical();
//        horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();
//
//        pivot = getTurnPower(targetHeading, odometry.heading);
//
//        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);
//
//        double left_Front = (vertical + horizontal + pivot) / denominator;
//        double left_Back = (vertical - horizontal + pivot) / denominator;
//        double right_Front = (vertical - horizontal - pivot) / denominator;
//        double right_Back = (vertical + horizontal - pivot) / denominator;
//
//        drive.RF.setPower(right_Front);
//        drive.RB.setPower(right_Back);
//        drive.LF.setPower(left_Front);
//        drive.LB.setPower(left_Back);
//
//        return pathing;
//
//    }
//
//    public boolean followPathAuto(double targetHeading, Odometry odometry, Drivetrain drive, double errorMargin){
//
//        odometry.update();
//
//        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);
//
//        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());
//
//        boolean pathing = true;
//
//        robotPositionVector.set(odometry.X, odometry.Y);
//
//        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < errorMargin && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < errorMargin && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
//            pathing = false;
//        }
//
//        boolean closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 10 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 10;
//
//        PathingPower correctivePower = new PathingPower();
//        PathingPower pathingPower;
//
//        if(Math.abs(odometry.getHorizontalVelocity()) < 3){
//            yI += 0.05;
//        }else {
//            yI = 0;
//        }
//
//        if(Math.abs(odometry.getVerticalVelocity()) < 3){
//            xI += 0.05;
//        }else {
//            xI = 0;
//        }
//
//        double heading = odometry.heading;
//
//        int closestPos = pathfollow.getClosestPositionOnPath(robotPositionVector);
//
//        if (pathfollow.followablePath.size()-1 > 700){
//            if (closestPos >= pathfollow.followablePath.size()-200){
//                gotToEnd = true;
//            } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
//                gotToEnd = false;
//            }
//        }else {
//            if (closestPos >= pathfollow.followablePath.size()-150){
//                gotToEnd = true;
//            } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
//                gotToEnd = false;
//            }
//        }
//
//        if (!gotToEnd){
//            pathingPower = getFullPathingPower(robotPositionVector, heading);
//            System.out.println(closestPos);
//            System.out.println(pathfollow.followablePath.size()-1);
//        }else {
//            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
//            pathingPower = new PathingPower(0,0);
//        }
//
//        vertical = correctivePower.getVertical() + pathingPower.getVertical();
//        horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();
//
//        pivot = getTurnPower(targetHeading, odometry.heading);
//
//        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);
//
//        double left_Front = (vertical + horizontal + pivot) / denominator;
//        double left_Back = (vertical - horizontal + pivot) / denominator;
//        double right_Front = (vertical - horizontal - pivot) / denominator;
//        double right_Back = (vertical + horizontal - pivot) / denominator;
//
//        drive.RF.setPower(right_Front);
//        drive.RB.setPower(right_Back);
//        drive.LF.setPower(left_Front);
//        drive.LB.setPower(left_Back);
//
//        return pathing;
//    }

    public boolean followPathAuto(double targetHeading, Odometry odometry, Drivetrain drive, Telemetry telemetry){

        odometry.update();

        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean pathing = true;

        robotPositionVector.set(odometry.X, odometry.Y);

        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
            pathing = false;
        }

        boolean closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 10 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 10;

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        if(Math.abs(odometry.getHorizontalVelocity()) < 3){
            yI += 0.005;
        }else {
            yI = 0;
        }

        if(Math.abs(odometry.getVerticalVelocity()) < 3){
            xI += 0.005;
        }else {
            xI = 0;
        }

        double heading = odometry.heading;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPositionVector);

        if (pathfollow.followablePath.size()-1 > 700){
            if (closestPos >= pathfollow.followablePath.size()-700){
                gotToEnd = true;
            } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
                gotToEnd = false;
            }
        }else {
            if (closestPos >= pathfollow.followablePath.size()-350){
                gotToEnd = true;
            } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
                gotToEnd = false;
            }
        }

        if (!gotToEnd){
            pathingPower = getFullPathingPower(robotPositionVector, heading, odometry);
        }else {
            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
            pathingPower = new PathingPower(0,0);
        }

        telemetry.addData("pointX", getPointOnPath(closestPos).getX());
        telemetry.addData("pointY", getPointOnPath(closestPos).getY());
        telemetry.addData("pointY", heading);

        vertical = correctivePower.getVertical() + pathingPower.getVertical();
        horizontal = -(correctivePower.getHorizontal() + pathingPower.getHorizontal());
        pivot = getTurnPower(targetHeading, odometry.heading);


        telemetry.addData("vertical", vertical);
        telemetry.addData("horizontal", horizontal);
        telemetry.update();

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        double left_Front = (vertical + horizontal + pivot) / denominator;
        double left_Back = (vertical - horizontal + pivot) / denominator;
        double right_Front = (vertical - horizontal - pivot) / denominator;
        double right_Back = (vertical + horizontal - pivot) / denominator;

//        drive.RF.setPower(right_Front);
//        drive.RB.setPower(right_Back);
//        drive.LF.setPower(left_Front);
//        drive.LB.setPower(left_Back);

        return pathing;

    }

    public boolean followPathAuto(double targetHeading, Odometry odometry, Drivetrain drive){

        odometry.update();

        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean pathing = true;

        robotPositionVector.set(odometry.X, odometry.Y);

        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
            pathing = false;
        }

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        if(Math.abs(odometry.getHorizontalVelocity()) < 3){
            yI += 0.05;
        }else {
            yI = 0;
        }

        if(Math.abs(odometry.getVerticalVelocity()) < 3){
            xI += 0.05;
        }else {
            xI = 0;
        }

        double heading = odometry.heading;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPositionVector);

        if (closestPos >= pathfollow.followablePath.size()-50){
            gotToEnd = true;
        } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
            gotToEnd = false;
        }

        if (!gotToEnd){
            pathingPower = getFullPathingPower(robotPositionVector, heading, odometry);
            vertical = pathingPower.getVertical();
            horizontal = -(pathingPower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading, 0.02, 0.005);
        }else {
            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
            vertical = correctivePower.getVertical();
            horizontal = -(correctivePower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading, 0.01, 0.005);
        }

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        System.out.println("vertical before set" + vertical);

        double left_Front = (vertical + horizontal + pivot) / denominator;
        double left_Back = (vertical - horizontal + pivot) / denominator;
        double right_Front = (vertical - horizontal - pivot) / denominator;
        double right_Back = (vertical + horizontal - pivot) / denominator;

        drive.RF.setPower(right_Front);
        drive.RB.setPower(right_Back);
        drive.LF.setPower(left_Front);
        drive.LB.setPower(left_Back);

        return pathing;

    }

    public boolean followPathAutoHeading(double targetHeading, Odometry odometry, Drivetrain drive, double p){

        odometry.update();

        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean pathing = true;

        robotPositionVector.set(odometry.X, odometry.Y);

        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
            pathing = false;
        }

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        if(Math.abs(odometry.getHorizontalVelocity()) < 3){
            yI += 0.05;
        }else {
            yI = 0;
        }

        if(Math.abs(odometry.getVerticalVelocity()) < 3){
            xI += 0.05;
        }else {
            xI = 0;
        }

        double heading = odometry.heading;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPositionVector);

        if (closestPos >= pathfollow.followablePath.size()-50){
            gotToEnd = true;
        } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
            gotToEnd = false;
        }

        if (!gotToEnd){
            pathingPower = getFullPathingPower(robotPositionVector, heading, odometry);
            vertical = pathingPower.getVertical();
            horizontal = -(pathingPower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading, p, 0.002);
        }else {
            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
            vertical = correctivePower.getVertical();
            horizontal = -(correctivePower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading, 0.008, 0.002);
        }

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        System.out.println("vertical before set" + vertical);

        double left_Front = (vertical + horizontal + pivot) / denominator;
        double left_Back = (vertical - horizontal + pivot) / denominator;
        double right_Front = (vertical - horizontal - pivot) / denominator;
        double right_Back = (vertical + horizontal - pivot) / denominator;

        drive.RF.setPower(right_Front);
        drive.RB.setPower(right_Back);
        drive.LF.setPower(left_Front);
        drive.LB.setPower(left_Back);

        return pathing;

    }

    public boolean followPathAutoHeading(double targetHeading, Odometry odometry, Drivetrain drive, double p, double errorMargin){

        odometry.update();

        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean pathing = true;

        robotPositionVector.set(odometry.X, odometry.Y);

        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < errorMargin && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < errorMargin && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
            pathing = false;
        }

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        if(Math.abs(odometry.getHorizontalVelocity()) < 3){
            yI += 0.05;
        }else {
            yI = 0;
        }

        if(Math.abs(odometry.getVerticalVelocity()) < 3){
            xI += 0.05;
        }else {
            xI = 0;
        }

        double heading = odometry.heading;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPositionVector);

        if (closestPos >= pathfollow.followablePath.size()-50){
            gotToEnd = true;
        } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
            gotToEnd = false;
        }

        if (!gotToEnd){
            pathingPower = getFullPathingPower(robotPositionVector, heading, odometry);
            vertical = pathingPower.getVertical();
            horizontal = -(pathingPower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading, p, 0.002);
        }else {
            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
            vertical = correctivePower.getVertical();
            horizontal = -(correctivePower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading, 0.008, 0.002);
        }

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        System.out.println("vertical before set" + vertical);

        double left_Front = (vertical + horizontal + pivot) / denominator;
        double left_Back = (vertical - horizontal + pivot) / denominator;
        double right_Front = (vertical - horizontal - pivot) / denominator;
        double right_Back = (vertical + horizontal - pivot) / denominator;

        drive.RF.setPower(right_Front);
        drive.RB.setPower(right_Back);
        drive.LF.setPower(left_Front);
        drive.LB.setPower(left_Back);

        return pathing;

    }

    public boolean followPathAuto(double targetHeading, Odometry odometry, Drivetrain drive, double errorMargin){

        odometry.update();

        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean pathing = true;

        robotPositionVector.set(odometry.X, odometry.Y);

        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < errorMargin && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < errorMargin && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
            pathing = false;
        }

        boolean closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 10 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 10;

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        if(Math.abs(odometry.getHorizontalVelocity()) < 3){
            yI += 0.05;
        }else {
            yI = 0;
        }

        if(Math.abs(odometry.getVerticalVelocity()) < 3){
            xI += 0.05;
        }else {
            xI = 0;
        }

        double heading = odometry.heading;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPositionVector);

        if (closestPos >= pathfollow.followablePath.size()-50){
            gotToEnd = true;
        } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
            gotToEnd = false;
        }

        if (!gotToEnd){
            pathingPower = getFullPathingPower(robotPositionVector, heading, odometry);
            vertical = pathingPower.getVertical();
            horizontal = -(pathingPower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading);
        }else {
            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
            vertical = correctivePower.getVertical();
            horizontal = -(correctivePower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading);
        }



        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        System.out.println("vertical before set" + vertical);

        double left_Front = (vertical + horizontal + pivot) / denominator;
        double left_Back = (vertical - horizontal + pivot) / denominator;
        double right_Front = (vertical - horizontal - pivot) / denominator;
        double right_Back = (vertical + horizontal - pivot) / denominator;

        drive.RF.setPower(right_Front);
        drive.RB.setPower(right_Back);
        drive.LF.setPower(left_Front);
        drive.LB.setPower(left_Back);

        return pathing;

    }

    public boolean followPathAuto(double targetHeading, Odometry odometry, Drivetrain drive, double errorMargin, int correctionDistance){

        odometry.update();

        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean pathing = true;

        robotPositionVector.set(odometry.X, odometry.Y);

        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < errorMargin && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < errorMargin && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
            pathing = false;
        }

        boolean closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 10 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 10;

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        if(Math.abs(odometry.getHorizontalVelocity()) < 3){
            yI += 0.05;
        }else {
            yI = 0;
        }

        if(Math.abs(odometry.getVerticalVelocity()) < 3){
            xI += 0.05;
        }else {
            xI = 0;
        }

        double heading = odometry.heading;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPositionVector);

        if (closestPos >= pathfollow.followablePath.size()-correctionDistance){
            gotToEnd = true;
        } else if (closestPos < (pathfollow.followablePath.size()-1)/2) {
            gotToEnd = false;
        }

        if (!gotToEnd){
            pathingPower = getFullPathingPower(robotPositionVector, heading, odometry);
            vertical = pathingPower.getVertical();
            horizontal = -(pathingPower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading);
        }else {
            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
            vertical = correctivePower.getVertical();
            horizontal = -(correctivePower.getHorizontal());
            pivot = getTurnPower(targetHeading, odometry.heading);
        }



        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        System.out.println("vertical before set" + vertical);

        double left_Front = (vertical + horizontal + pivot) / denominator;
        double left_Back = (vertical - horizontal + pivot) / denominator;
        double right_Front = (vertical - horizontal - pivot) / denominator;
        double right_Back = (vertical + horizontal - pivot) / denominator;

        drive.RF.setPower(right_Front);
        drive.RB.setPower(right_Back);
        drive.LF.setPower(left_Front);
        drive.LB.setPower(left_Back);

        return pathing;

    }

    public Vector2D getLastPoint(){
        return pathfollow.getPointOnFollowable(pathfollow.getLastPoint());
    }

    public double getTurnError(double targetHeading, double currentHeading){

        double turnError;

        turnError = targetHeading - currentHeading;

        return Math.abs(turnError);
    }

    public double getTurnPowerTeleop(double targetHeading, double currentHeading){

        double turnPower;

        double rotdist = (targetHeading - currentHeading);

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 180) {
            rotdist = (rotdist - 360);
        }

        headingPID = new PIDController(0.03, 0, rotationD);

        turnPower = headingPID.calculate(-rotdist);

        return turnPower*1.3;
    }

    public double getTurnPower(double targetHeading, double currentHeading){

        double turnPower;

        double rotdist = (targetHeading - currentHeading);

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 180) {
            rotdist = (rotdist - 360);
        }

        headingPID = new PIDController(rotationP, 0, rotationD);

        turnPower = headingPID.calculate(-rotdist);

        return turnPower;
    }

    public double getTurnPower(double targetHeading, double currentHeading, double P, double D){

        double turnPower;

        double rotdist = (targetHeading - currentHeading);

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 180) {
            rotdist = (rotdist - 360);
        }

        headingPID = new PIDController(P, 0, D);

        turnPower = headingPID.calculate(-rotdist);

        return turnPower;
    }

    //still need to finish this
    public Vector2D getCurrentStopPosition(PathingVelocity currentVelocity){
        Vector2D stopPoint = new Vector2D();

        return null;
    }

    public double getErrorBetweenPoints(int index){
        Vector2D firstPoint;
        Vector2D secondPoint;

        firstPoint = new Vector2D(pathfollow.getPointOnFollowable(index).getX(), pathfollow.getPointOnFollowable(index).getY());
        index++;
        secondPoint = new Vector2D(pathfollow.getPointOnFollowable(index).getX(), pathfollow.getPointOnFollowable(index).getY());

        return Math.hypot(firstPoint.getX() - secondPoint.getX(), firstPoint.getY() - secondPoint.getY());
    }

    public Vector2D getPointOnPath(int index){
        return pathfollow.getPointOnFollowable(index);
    }

    public Vector2D getPoint(Vector2D robotPos){
        return pathfollow.getPointOnFollowable(pathfollow.getClosestPositionOnPath(robotPos));
    }

    public int getIndex(Vector2D robotPos){
        return pathfollow.getClosestPositionOnPath(robotPos);
    }

    public double getPathLength(){
        return this.pathfollow.calculateTotalDistance();
    }

    public double getSectionLength(Vector2D robotPos, Vector2D endpoint){
        return this.pathfollow.calculateDistancePointToPoint(robotPos, endpoint);
    }

    public double calculateDistance(Vector2D point1, Vector2D point2) {
        double dx = point2.getX() - point1.getX();
        double dy = point2.getY() - point1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
