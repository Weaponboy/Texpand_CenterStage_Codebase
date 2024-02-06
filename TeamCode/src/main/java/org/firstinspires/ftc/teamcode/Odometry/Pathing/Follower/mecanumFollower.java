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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Stack_2.CycleMethods;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingPower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;

import java.util.ArrayList;
import java.util.List;

@Config
public class mecanumFollower {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    double yI = 0;
    double xI = 0;

    public static double X = 0;
    public static double Y = 0;

    double loopTime;
    int counter;
    double lastLoopTime;

    ElapsedTime elapsedTime = new ElapsedTime();

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

    public PathingPower getFullPathingPower(Vector2D robotPos, double heading){

        XCorrective.setPID(driveP, 0, driveD);
        YCorrective.setPID(strafeP, 0, strafeD);

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        PathingPower pathingPower;
        PathingPower actualPathingPower = new PathingPower();

        double ky = 0.0234;
        double kx = 0.0154;

        PathingVelocity pathingVelocity;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);

        pathingVelocity = pathfollow.getTargetVelocity(closestPos);//try this

        error = pathfollow.getErrorToPath(robotPos, closestPos);

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPowerC = XCorrective.calculate(-robotRelativeXError)*1.2;
        double yPowerC = YCorrective.calculate(-robotRelativeYError)*1.4;

        double xPower = pathingVelocity.getYVelocity() * Math.sin(Math.toRadians(heading)) + pathingVelocity.getXVelocity() * Math.cos(Math.toRadians(heading));
        double yPower = pathingVelocity.getYVelocity() * Math.cos(Math.toRadians(heading)) - pathingVelocity.getXVelocity() * Math.sin(Math.toRadians(heading));

        vertical = kx * xPower;
        horizontal = ky * yPower;

        actualPathingPower.set(xPowerC+vertical, yPowerC+horizontal);

        return actualPathingPower;
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

    public PathingPower getCorrectivePowerAtEnd(Vector2D robotPos, Vector2D targetPos, double heading){

        XCorrective.setPID(0.02, xI, 0.0001);
        YCorrective.setPID(0.03, yI, 0.0001);

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

    public void TestNewPathingMethod(Odometry odometry){

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        Vector2D robotPos = new Vector2D();

        boolean reachedTarget = false;

        do {

            counter++;

            if (counter > 50){
                counter = 0;
                loopTime = elapsedTime.milliseconds() - lastLoopTime;
            }

            lastLoopTime = elapsedTime.milliseconds();

            odometry.update();

            robotPos.set(odometry.X, odometry.Y);

            if (Math.abs(robotPos.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPos.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3){
                reachedTarget = true;
            }

            Vector2D error = pathfollow.getPointOnFollowable(pathfollow.getClosestPositionOnPath(robotPos));

            dashboardTelemetry.addData("X odo", robotPos.getX());
            dashboardTelemetry.addData("Y odo", robotPos.getY());
            dashboardTelemetry.addData("X", error.getX());
            dashboardTelemetry.addData("Y", error.getY());
            dashboardTelemetry.addData("loop time", loopTime);
            dashboardTelemetry.update();

        }while(!reachedTarget);

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

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
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

    //deploy arm method
    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, Delivery delivery, Delivery_Slides deliverySlides) throws InterruptedException {

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

            if (odometry.X > 200 && odometry.getVerticalVelocity() > 5){

                deliverySlides.DeliverySlides(400, 0.8);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition());

            }

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
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

    //deploy arm method yellow pixel
    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, Delivery delivery, Delivery_Slides deliverySlides, Collection collection) throws InterruptedException {

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

            if (odometry.X > 260 && odometry.getVerticalVelocity() > 5){

                collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
                collection.updateIntakeHeight();

                sleep(200);

                deliverySlides.DeliverySlides(220, 0.6);

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition());

            }

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
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

    //deploy arm method yellow pixel with heading change
    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, Vector2D pointForHeadingChange, double secondHeading, Delivery delivery, Delivery_Slides deliverySlides, Collection collection) throws InterruptedException {

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

            if (odometry.X > 260 && odometry.getVerticalVelocity() > 5){

                collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
                collection.updateIntakeHeight();

                sleep(200);

                deliverySlides.DeliverySlides(220, 0.6);

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition());

            }

            if (Math.abs(pointForHeadingChange.getX() - odometry.X) < 15 && Math.abs(pointForHeadingChange.getY() - odometry.Y) < 15){
                targetHeading = secondHeading;
            }

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
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

    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, Telemetry telemetry){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean reachedTarget = false;

        boolean closeToTarget = false;

        xI = 0;
        yI = 0;

        do {

            counter++;

            if (counter > 50){
                counter = 0;
                loopTime = elapsedTime.milliseconds() - lastLoopTime;
            }

            lastLoopTime = elapsedTime.milliseconds();

            odometry.update();

            robotPositionVector.set(odometry.X, odometry.Y);

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
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

            telemetry.addData("loop time", loopTime);
            telemetry.update();

        }while(!reachedTarget);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, String shortPath){

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

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
                reachedTarget = true;
            }

            PathingPower correctivePower;
            PathingPower pathingPower;

            closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 18 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 18;

            if(Math.abs(odometry.getHorizontalVelocity()) < 2){
                yI += 0.005;
            }else {
                yI = 0;
            }

            if(Math.abs(odometry.getVerticalVelocity()) < 2){
                xI += 0.005;
            }else {
                xI = 0;
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

    //turn intake on method
    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, Collection collection, Vector2D pointToTurnOn){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean reachedTarget = false;

        boolean closeToTarget = false;

        xI = 0;
        yI = 0;

        do {

            if (Math.abs(pointToTurnOn.getX() - odometry.X) < 20 && Math.abs(pointToTurnOn.getY() - odometry.Y) < 20){
                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();
            }

            odometry.update();

            robotPositionVector.set(odometry.X, odometry.Y);

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
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

            if (Math.abs(pointForHeadingChange.getX() - odometry.X) < 15 && Math.abs(pointForHeadingChange.getY() - odometry.Y) < 15){
                targetHeading = secondHeading;
            }

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
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

    //change heading mid path
    public void followPath(double targetHeading, Odometry odometry, Drivetrain drive, Vector2D pointForHeadingChange, double secondHeading, List<LynxModule> allHubs){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(odometry.X, odometry.Y);

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean reachedTarget = false;

        boolean closeToTarget = false;

        xI = 0;
        yI = 0;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        do {

            loopTime = elapsedTime.milliseconds() - lastLoopTime;

            lastLoopTime = elapsedTime.milliseconds();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            odometry.update();

            robotPositionVector.set(odometry.X, odometry.Y);

            if (Math.abs(pointForHeadingChange.getX() - odometry.X) < 10 && Math.abs(pointForHeadingChange.getY() - odometry.Y) < 10){
                targetHeading = secondHeading;
            }

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4 && Math.abs(odometry.getVerticalVelocity()) < 3 && Math.abs(odometry.getHorizontalVelocity()) < 3 && Math.abs(targetHeading - odometry.heading) < 2){
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

            RobotLog.d("loop time: " + loopTime);

//            dashboardTelemetry.addData("time", time);
//            dashboardTelemetry.addData("loop time", loopTime);
//            dashboardTelemetry.update();

        }while(!reachedTarget);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public void testingLoopTime(double targetHeading, Vector2D robotPos, double heading){

        //for getting pathing power and corrective as well
        Vector2D robotPositionVector = new Vector2D(robotPos.getX(), robotPos.getY());

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        boolean reachedTarget = false;

        boolean closeToTarget = false;

        xI = 0;
        yI = 0;

        do {

            loopTime = elapsedTime.milliseconds() - lastLoopTime;

            lastLoopTime = elapsedTime.milliseconds();

            robotPositionVector.set(X, Y);

            if (Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 1.4 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 1.4){
                reachedTarget = true;
            }

            PathingPower correctivePower = new PathingPower();
            PathingPower pathingPower;

            closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 10 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 10;

            getCorrective.reset();

            if (!closeToTarget){
                pathingPower = getFullPathingPower(robotPositionVector, heading);
            }else {
                correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
                pathingPower = new PathingPower(0,0);
            }

            vertical = correctivePower.getVertical() + pathingPower.getVertical();
            horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();

            pivot = getTurnPower(targetHeading, heading);

            time = getCorrective.milliseconds();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("X", X);
            packet.put("Y", Y);
            dashboard.sendTelemetryPacket(packet);

            dashboardTelemetry.addData("time", loopTime);
            dashboardTelemetry.update();

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

            double left_Front = (vertical + horizontal + pivot) / denominator;
            double left_Back = (vertical - horizontal + pivot) / denominator;
            double right_Front = (vertical - horizontal - pivot) / denominator;
            double right_Back = (vertical + horizontal - pivot) / denominator;

        }while(!reachedTarget);

    }

    public void basePower(Vector2D robotPositionVector, Vector2D targetPoint, boolean closeToTarget, Odometry odometry, double targetHeading){

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        closeToTarget = Math.abs(robotPositionVector.getX() - targetPoint.getX()) < 10 && Math.abs(robotPositionVector.getY() - targetPoint.getY()) < 10;

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

        getCorrective.reset();

        double heading = odometry.heading;

        if (!closeToTarget){
            pathingPower = getFullPathingPower(robotPositionVector, heading);
        }else {
            correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPoint, heading);
            pathingPower = new PathingPower(0,0);
        }

        vertical = correctivePower.getVertical() + pathingPower.getVertical();
        horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();

        pivot = getTurnPower(targetHeading, odometry.heading);

        time = getCorrective.milliseconds();

        dashboardTelemetry.addData("time", time);

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
        } else if (rotdist > 360) {
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
        } else if (rotdist > 360) {
            rotdist = (rotdist - 360);
        }

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
                dashboardTelemetry.addData("x", odometry.X);
                dashboardTelemetry.addData("y", odometry.Y);
                dashboardTelemetry.addData("heading", odometry.heading);
                dashboardTelemetry.addData("leftPod", odometry.leftPod.getCurrentPosition());
                dashboardTelemetry.addData("rightPod", odometry.rightPod.getCurrentPosition());
                dashboardTelemetry.addData("centerPod", odometry.centerPod.getCurrentPosition());
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
