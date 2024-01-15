package org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.Heading;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.X;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.Y;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.vertical;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.robotPos;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingPower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;

import java.util.ArrayList;

public class mecanumFollower {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    HardwareMap hardwareMap;

    PIDController XCorrective;
    PIDController YCorrective;
    PIDController headingPID;

    FollowPath pathfollow;

    double yI = 0;
    double xI = 0;

    public void setPath(ArrayList<Vector2D> trajectory, ArrayList<PathingVelocity> pathingVelocity){
        pathfollow = new FollowPath(trajectory, pathingVelocity);
    }

    public PathingPower getPathingPower(Vector2D robotPos, double heading){

        PathingPower pathingPower;
        PathingPower actualPathingPower = new PathingPower();

        double ky = 0.0234;
        double kx = 0.0154;

        PathingVelocity pathingVelocity;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);

        pathingVelocity = pathfollow.getTargetVelocity(closestPos);//try this

        double xPower = pathingVelocity.getYVelocity() * Math.sin(Math.toRadians(heading)) + pathingVelocity.getXVelocity() * Math.cos(Math.toRadians(heading));
        double yPower = pathingVelocity.getYVelocity() * Math.cos(Math.toRadians(heading)) - pathingVelocity.getXVelocity() * Math.sin(Math.toRadians(heading));

        vertical = kx * xPower;
        horizontal = ky * yPower;

        pathingPower = new PathingPower(vertical, horizontal);

        return pathingPower;
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

        XCorrective = new PIDController(driveP, 0, driveD);
        YCorrective = new PIDController(strafeP, 0, strafeD);

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

    public PathingPower getCorrectivePowerAtEnd(Vector2D robotPos, Vector2D targetPos, double heading){

        XCorrective = new PIDController(0.03, xI, 0.002);
        YCorrective = new PIDController(0.04, yI, 0.001);

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        error = new Vector2D( targetPos.getX() - robotPos.getX(),  targetPos.getY() - robotPos.getY());

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPower = XCorrective.calculate(-robotRelativeXError)*1.2;
        double yPower = YCorrective.calculate(-robotRelativeYError)*1.3;

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

    //normal method
    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean reachedTarget = false;

        boolean closeToTarget = false;

        xI = 0;
        yI = 0;

        do {

            odometry.update();

            robotPositionVector.set(odometry.X, odometry.Y);

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3){
                reachedTarget = true;
            }

            basePower(robotPositionVector, targetPoint, closeToTarget, odometry, targetHeading);

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

            double left_Front = (vertical + horizontal + pivot) / denominator;
            double left_Back = (vertical - horizontal + pivot) / denominator;
            double right_Front = (vertical - horizontal - pivot) / denominator;
            double right_Back = (vertical + horizontal - pivot) / denominator;

            drive.RF.setPower(right_Front);
            drive.RB.setPower(right_Back);
            drive.LF.setPower(left_Front);
            drive.LB.setPower(left_Back);

            dashboardTelemetry.addData("close to target", closeToTarget);
            dashboardTelemetry.update();

        }while(!reachedTarget);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    //turn intake on method
    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, Collection collection, Vector2D pointToTurnOn){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        int lastIndex = pathfollow.getLastPoint();

        boolean reachedTarget = false;

        boolean closeToTarget = false;

        xI = 0;
        yI = 0;

        do {

            odometry.update();

            robotPositionVector.set(odometry.X, odometry.Y);

            if (Math.abs(pointToTurnOn.getX() - odometry.X) < 10 && Math.abs(pointToTurnOn.getY() - odometry.Y) < 10){
                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();
            }

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.2 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.2 && odometry.getVerticalVelocity() < 3 && odometry.getHorizontalVelocity() < 3){
                reachedTarget = true;
            }

            basePower(robotPositionVector, targetPoint, closeToTarget, odometry, targetHeading);

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

            double left_Front = (vertical + horizontal + pivot) / denominator;
            double left_Back = (vertical - horizontal + pivot) / denominator;
            double right_Front = (vertical - horizontal - pivot) / denominator;
            double right_Back = (vertical + horizontal - pivot) / denominator;

            drive.RF.setPower(right_Front);
            drive.RB.setPower(right_Back);
            drive.LF.setPower(left_Front);
            drive.LB.setPower(left_Back);

            dashboardTelemetry.addData("close to target", closeToTarget);
            dashboardTelemetry.update();

        }while(!reachedTarget);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    //change heading mid path
    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, Vector2D pointForHeadingChange, double secondHeading){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean reachedTarget = false;

        boolean closeToTarget = false;

        xI = 0;
        yI = 0;

        do {

            odometry.update();

            robotPositionVector.set(odometry.X, odometry.Y);

            if (Math.abs(pointForHeadingChange.getX() - odometry.X) < 10 && Math.abs(pointForHeadingChange.getY() - odometry.Y) < 10){
                targetHeading = secondHeading;
            }

            //use follower methods to get motor power
            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3){
                reachedTarget = true;
            }

            basePower(robotPositionVector, targetPoint, closeToTarget, odometry, targetHeading);

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

            double left_Front = (vertical + horizontal + pivot) / denominator;
            double left_Back = (vertical - horizontal + pivot) / denominator;
            double right_Front = (vertical - horizontal - pivot) / denominator;
            double right_Back = (vertical + horizontal - pivot) / denominator;

            drive.RF.setPower(right_Front);
            drive.RB.setPower(right_Back);
            drive.LF.setPower(left_Front);
            drive.LB.setPower(left_Back);

        }while(!reachedTarget);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public void basePower(Vector2D robotPositionVector, Vector2D targetPoint, boolean closeToTarget, Odometry odometry, double targetHeading){

        PathingPower correctivePower;
        PathingPower pathingPower;

        closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 5 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 5;

        if(!reachedTarget && Math.abs(odometry.getHorizontalVelocity) < 1){
            yI += 1;
        }else if(!reachedTarget && Math.abs(odometry.getVerticalVelocity) < 1){
            xI += 1;
        }

        if (!closeToTarget){
            pathingPower = getPathingPower(robotPositionVector, odometry.heading);
            correctivePower = getCorrectivePowerOnPath(robotPositionVector, odometry.heading);
        }else {
            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, odometry.heading);
            pathingPower = new PathingPower(0,0);
        }

        vertical = correctivePower.getVertical() + pathingPower.getVertical();
        horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();

        pivot = getTurnPower(targetHeading, odometry.heading);

    }

    public boolean followPathTeleop(boolean power, double targetHeading, boolean debugging, Odometry odometry, Drivetrain drive, Telemetry dashboardTelemetry){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        Vector2D targetPointMiddle = pathfollow.getPointOnFollowable(pathfollow.getClosestPositionOnPath(robotPositionVector));

        PathingVelocity targetvelo = pathfollow.getTargetVelocity(1);

        int lastIndex = pathfollow.getLastPoint();

        boolean busyPathing = true;

        boolean closeToTarget = false;

        String pathing;

        boolean reachedTarget = false;

        int counter = 0;

        odometry.update();

        robotPositionVector.set(odometry.X, odometry.Y);

        if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.2 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.2 && odometry.getVerticalVelocity() < 3 && odometry.getHorizontalVelocity() < 3){
            reachedTarget = true;
            busyPathing = false;
        }

        closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 5 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 5;

        PathingPower correctivePower;
        PathingPower pathingPower;

        if (!closeToTarget){
            pathingPower = getPathingPower(robotPositionVector, odometry.heading);
            correctivePower = getCorrectivePowerOnPath(robotPositionVector, odometry.heading);
        }else {
            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, odometry.heading);
            pathingPower = new PathingPower(0,0);
        }

        vertical = correctivePower.getVertical() + pathingPower.getVertical();
        horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();

        pivot = getTurnPower(targetHeading, odometry.heading);

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        double left_Front = 0;
        double left_Back = 0;
        double right_Front = 0;
        double right_Back = 0;

        if (power){
            left_Front = (vertical + horizontal + pivot) / denominator;
            left_Back = (vertical - horizontal + pivot) / denominator;
            right_Front = (vertical - horizontal - pivot) / denominator;
            right_Back = (vertical + horizontal - pivot) / denominator;
        }

        drive.RF.setPower(right_Front);
        drive.RB.setPower(right_Back);
        drive.LF.setPower(left_Front);
        drive.LB.setPower(left_Back);


        if (debugging){
            dashboardTelemetry.addData("counter", counter);

            dashboardTelemetry.addData("busyPathing", busyPathing);
            dashboardTelemetry.addData("reachedTarget", reachedTarget);
            dashboardTelemetry.addLine();
            dashboardTelemetry.addData("heading", odometry.heading);
            dashboardTelemetry.addData("target pos", targetPoint);
            dashboardTelemetry.addData("target pos middle", targetPointMiddle);
            dashboardTelemetry.addData("robotPos", robotPositionVector);

            dashboardTelemetry.addLine();
            dashboardTelemetry.addData("target velo x", targetvelo.getXVelocity());
            dashboardTelemetry.addData("target velo y", targetvelo.getYVelocity());

            dashboardTelemetry.addLine();
            dashboardTelemetry.addData("vertical", Math.abs(pathingPower.getVertical()));
            dashboardTelemetry.addData("horizontal", Math.abs(pathingPower.getHorizontal()));
        }else {
            dashboardTelemetry.addData("x", odometry.X);
            dashboardTelemetry.addData("y", odometry.Y);
            dashboardTelemetry.addData("heading", odometry.heading);
        }

        return busyPathing;
    }

    public Vector2D getLastPoint(){
        return pathfollow.getPointOnFollowable(pathfollow.getLastPoint());
    }

    //test Method
    public robotPos followPath(robotPos robotpos, boolean power, double targetHeading, boolean debugging, Odometry odometry){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(robotpos.getX(), robotpos.getY());

        robotPos robotPosForReset = new robotPos(robotpos.getX(), robotpos.getY(), robotpos.getHeading());

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        Vector2D targetPointMiddle = pathfollow.getPointOnFollowable(pathfollow.getClosestPositionOnPath(robotPositionVector));

        PathingVelocity targetvelo = pathfollow.getTargetVelocity(1);

        int lastIndex = pathfollow.getLastPoint();

        boolean busyPathing = true;

        String pathing;

        boolean busyCorrecting;

        int counter = 0;

        do {

            odometry.update(1);

            counter++;

            if (counter == 1000){
                X = targetPoint.getX()-0.6;
                Y = targetPoint.getY()-0.6;
                Heading = 180;
            }

            robotPositionVector.set(X, Y);

            //use follower methods to get motor power
            PathingPower correctivePower;
            correctivePower = getCorrectivePowerOnPath(robotPositionVector, Heading);

            Vector2D correctivePosition;
            correctivePosition = getCorrectivePosition(robotPositionVector);

            PathingPower pathingPower;
            pathingPower = getPathingPower(robotPositionVector, Heading);

            //apply motor power in order of importance
            if (Math.abs(correctivePosition.getX()) > 5 || Math.abs(correctivePosition.getY()) > 5 && busyPathing) {
                vertical = correctivePower.getVertical();
                horizontal = correctivePower.getHorizontal();
                pathing = "Corrective on path";
            } else if (!busyPathing) {
                vertical = correctivePower.getVertical();
                horizontal = correctivePower.getHorizontal();
                pathing = "Corrective at end";
            } else {
                horizontal = pathingPower.getHorizontal();
                vertical = pathingPower.getVertical();
                pathing = "pathing";
            }

            if (Math.abs(pathingPower.getHorizontal()) < 0.08 && Math.abs(pathingPower.getVertical()) < 0.08){
                busyPathing = false;
            }else {
                busyPathing = true;
            }

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 0.8 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 0.8  && !busyPathing){
                busyCorrecting = false;
            }else {
                busyCorrecting = true;
            }

            pivot = getTurnPower(targetHeading, Heading);

            if (debugging){
                dashboardTelemetry.addData("counter", counter);

                dashboardTelemetry.addData("busyPathing", busyPathing);
                dashboardTelemetry.addData("busyCorrecting", busyCorrecting);
                dashboardTelemetry.addLine();
                dashboardTelemetry.addData("heading", Heading);
                dashboardTelemetry.addData("target pos", targetPoint);
                dashboardTelemetry.addData("target pos middle", targetPointMiddle);
                dashboardTelemetry.addData("robotPos", robotPositionVector);

                dashboardTelemetry.addLine();
                dashboardTelemetry.addData("target velo x", targetvelo.getXVelocity());
                dashboardTelemetry.addData("target velo y", targetvelo.getYVelocity());

                dashboardTelemetry.addLine();
                dashboardTelemetry.addData("vertical", Math.abs(pathingPower.getVertical()));
                dashboardTelemetry.addData("horizontal", Math.abs(pathingPower.getHorizontal()));
                dashboardTelemetry.addData("power", pathing);
                dashboardTelemetry.update();
            }else {
                dashboardTelemetry.addData("x", odometry.X);
                dashboardTelemetry.addData("y", odometry.Y);
                dashboardTelemetry.addData("heading", odometry.heading);
                dashboardTelemetry.update();
            }

        }while(busyCorrecting);

        return robotPosForReset;

    }

    public double getTurnError(double targetHeading, double currentHeading){

        double turnError;

        turnError = targetHeading - currentHeading;

        return Math.abs(turnError);
    }

    public double getTurnPower(double targetHeading, double currentHeading){

        double turnPower;

        double rotdist = (targetHeading - currentHeading);

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 360) {
            rotdist = (rotdist - 360);
        }

//        rotdist = Math.toRadians(rotdist);

        headingPID = new PIDController(rotationP, 0, rotationD);

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

    public double calculateDistance(Vector2D point1, Vector2D point2) {
        double dx = point2.getX() - point1.getX();
        double dy = point2.getY() - point1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    //debugging method
    public void followPath(boolean power, double targetHeading, boolean debugging, Odometry odometry, Drivetrain drive, Telemetry dashboardTelemetry){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        Vector2D targetPointMiddle = pathfollow.getPointOnFollowable(pathfollow.getClosestPositionOnPath(robotPositionVector));

        PathingVelocity targetvelo = pathfollow.getTargetVelocity(1);

        int lastIndex = pathfollow.getLastPoint();

        boolean busyPathing = true;

        String pathing;

        boolean reachedTarget = false;

        int counter = 0;

        do {

            odometry.update();

            robotPositionVector.set(odometry.X, odometry.Y);

            //use follower methods to get motor power
            PathingPower correctivePower;
            correctivePower = getCorrectivePowerOnPath(robotPositionVector, odometry.heading);

            Vector2D correctivePosition;
            correctivePosition = getCorrectivePosition(robotPositionVector);

            PathingPower pathingPower;

            if (!reachedTarget){
                pathingPower = getPathingPower(robotPositionVector, odometry.heading);
            }else {
                pathingPower = new PathingPower(0,0);
            }

            vertical = correctivePower.getVertical() + pathingPower.getVertical();
            horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.2 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.2 && !reachedTarget){
                reachedTarget = true;
            }else {
                reachedTarget = false;
            }

            pivot = getTurnPower(targetHeading, odometry.heading);

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

            double left_Front = (vertical + horizontal + pivot) / denominator;
            double left_Back = (vertical - horizontal + pivot) / denominator;
            double right_Front = (vertical - horizontal - pivot) / denominator;
            double right_Back = (vertical + horizontal - pivot) / denominator;

            if (power){
                drive.RF.setPower(right_Front);
                drive.RB.setPower(right_Back);
                drive.LF.setPower(left_Front);
                drive.LB.setPower(left_Back);
            }

            if (debugging){
                dashboardTelemetry.addData("counter", counter);

                dashboardTelemetry.addData("busyPathing", busyPathing);
                dashboardTelemetry.addData("reachedTarget", reachedTarget);
                dashboardTelemetry.addLine();
                dashboardTelemetry.addData("heading", odometry.heading);
                dashboardTelemetry.addData("target pos", targetPoint);
                dashboardTelemetry.addData("target pos middle", targetPointMiddle);
                dashboardTelemetry.addData("robotPos", robotPositionVector);

                dashboardTelemetry.addLine();
                dashboardTelemetry.addData("target velo x", targetvelo.getXVelocity());
                dashboardTelemetry.addData("target velo y", targetvelo.getYVelocity());

                dashboardTelemetry.addLine();
                dashboardTelemetry.addData("vertical", pathingPower.getVertical());
                dashboardTelemetry.addData("horizontal", pathingPower.getHorizontal());
                dashboardTelemetry.update();
            }else {
                dashboardTelemetry.addData("x", odometry.X);
                dashboardTelemetry.addData("y", odometry.Y);
                dashboardTelemetry.addData("heading", odometry.heading);
                dashboardTelemetry.update();
            }

        }while(reachedTarget);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    //velocity crash saving
    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, boolean crashSafe){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        Vector2D targetPointMiddle = pathfollow.getPointOnFollowable(pathfollow.getClosestPositionOnPath(robotPositionVector));

        PathingVelocity targetvelo = pathfollow.getTargetVelocity(1);

        int lastIndex = pathfollow.getLastPoint();

        boolean busyPathing = true;

        String pathing;

        boolean reachedTarget = false;

        int counter = 0;

        do {

            odometry.update();

            robotPositionVector.set(odometry.X, odometry.Y);

            PathingPower correctivePower;
            Vector2D correctivePosition;

            if(crashSafe){

                correctivePower = getCorrectivePowerOnPath(robotPositionVector, odometry.heading);

                correctivePosition = getCorrectivePosition(robotPositionVector);
            }else {

                correctivePower = getCorrectivePowerOnPath(robotPositionVector, odometry.heading);

                correctivePosition = getCorrectivePosition(robotPositionVector);
            }



            PathingPower pathingPower;

            if (!reachedTarget){
                pathingPower = getPathingPower(robotPositionVector, odometry.heading);
            }else {
                pathingPower = new PathingPower(0,0);
            }

            vertical = correctivePower.getVertical() + pathingPower.getVertical();
            horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.2 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.2 && !reachedTarget){
                reachedTarget = true;
            }else {
                reachedTarget = false;
            }

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


        }while(reachedTarget);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

}
