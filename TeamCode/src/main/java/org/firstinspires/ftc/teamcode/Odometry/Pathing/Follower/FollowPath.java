package org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.maxYAcceleration;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.velocityDecreasePerPoint;
import static org.firstinspires.ftc.teamcode.hardware._.Odometry.getMaxVelocity;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingVelocity;

import java.util.ArrayList;
import java.util.List;

public class FollowPath {

    ArrayList<Vector2D> followablePath = new ArrayList<>();

    ArrayList<PathingVelocity> pathingVelocity = new ArrayList<>();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    int lastPointOnPath;

    public FollowPath(ArrayList<Vector2D> followablePath, ArrayList<PathingVelocity> pathingVelocity){
        this.followablePath = followablePath;
        this.pathingVelocity = pathingVelocity;
    }

    public double findAngle(PathingVelocity targetVelocity){

        double magnitude = Math.sqrt(targetVelocity.getXVelocity() * targetVelocity.getXVelocity() + targetVelocity.getYVelocity() * targetVelocity.getYVelocity());

        double radians = Math.atan2(targetVelocity.getYVelocity(), targetVelocity.getXVelocity());

        double degrees = Math.toDegrees(radians);

        if (degrees < 0) {
            degrees += 360;
        }

        return degrees;
    }

    public double acceleration_distance(){

        PathingVelocity pathVelo;

        double deltaTime;

        double decelerationNumber = 0;

        double pathLength = calculateTotalDistance();

        double acceleration_dt = (getMaxVelocity() * getMaxVelocity()) / (maxYAcceleration * 2);

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = pathLength / 2;

        if (acceleration_dt > halfway_distance){
            acceleration_dt = halfway_distance;
        }

        double new_max_velocity = ((acceleration_dt*0.25)*2.5);

        double deceleration_dt = acceleration_dt;

        double decIndex = deceleration_dt/2.5;

        double velocitySlope = new_max_velocity/getMaxVelocity();


        for (int i = 0; i < followablePath.size() - 1; i++) {

            if (i + decIndex >= followablePath.size()){

                velocitySlope -= velocityDecreasePerPoint;

                decelerationNumber = decelerationNumber/deceleration_dt;

                Vector2D currentPoint = followablePath.get(i);
                Vector2D nextPoint = followablePath.get(i + 1);

                double deltaX = nextPoint.getX() - currentPoint.getX();
                double deltaY = nextPoint.getY() - currentPoint.getY();

                deltaTime = Math.hypot(deltaY, deltaX) / getMaxVelocity();

                double velocityXValue = (deltaX / deltaTime) * decelerationNumber;
                double velocityYValue = (deltaY / deltaTime) * decelerationNumber;

                pathVelo = new PathingVelocity(velocityXValue, velocityYValue);

                pathingVelocity.add(pathVelo);

            }else {
                Vector2D currentPoint = followablePath.get(i);
                Vector2D nextPoint = followablePath.get(i + 1);

                decelerationNumber = velocitySlope;

                double deltaX = nextPoint.getX() - currentPoint.getX();
                double deltaY = nextPoint.getY() - currentPoint.getY();

                deltaTime = Math.hypot(deltaY, deltaX) / getMaxVelocity();

                double velocityXValue = (deltaX / deltaTime) * decelerationNumber;
                double velocityYValue = (deltaY / deltaTime) * decelerationNumber;

                pathVelo = new PathingVelocity(velocityXValue, velocityYValue);

                pathingVelocity.add(pathVelo);
            }

        }

        return new_max_velocity;

    }

    public void firstDerivative(){

        PathingVelocity pathVelo;

        double deltaTime;

        for (int i = 0; i < followablePath.size() - 1; i++) {
            Vector2D currentPoint = followablePath.get(i);
            Vector2D nextPoint = followablePath.get(i + 1);

            double deltaX = nextPoint.getX() - currentPoint.getX();
            double deltaY = nextPoint.getY() - currentPoint.getY();

            deltaTime = Math.hypot(deltaY, deltaX) / getMaxVelocity();

            double velocityXValue = deltaX / deltaTime;
            double velocityYValue = deltaY / deltaTime;

            pathVelo = new PathingVelocity(velocityXValue, velocityYValue);

            pathingVelocity.add(pathVelo);
        }
    }

    public Vector2D findPoint(){
        Vector2D point = new Vector2D();
        return point;
    }

    public double calculateTotalDistance() {

        double totalDistance = 0.0;

        for (int i = 0; i < followablePath.size() - 1; i++) {
            Vector2D point1 = followablePath.get(i);
            Vector2D point2 = followablePath.get(i + 1);
            totalDistance += calculateDistance(point1, point2);
        }
        return totalDistance;
    }

    public double calculateDistancePointToPoint(Vector2D robotPos, Vector2D endpoint) {

        int index = 0;

        int startIndex = getClosestPositionOnPath(robotPos);

        int endIndex = followablePath.size()-1;

        List<Vector2D> subList = followablePath.subList(startIndex, endIndex);

        ArrayList<Vector2D> sectionToLook = new ArrayList<>(subList);

        double totalDistance = 0.0;

        for (int i = 0; i < sectionToLook.size() - 1; i++) {
            Vector2D point1 = sectionToLook.get(i);
            Vector2D point2 = sectionToLook.get(i + 1);
            totalDistance += calculateDistance(point1, point2);
        }
        return totalDistance;
    }

    public double calculateDistance(Vector2D point1, Vector2D point2) {
        double dx = point2.getX() - point1.getX();
        double dy = point2.getY() - point1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public int getClosestPositionOnPath(Vector2D robotPos) {

        int index = 0;

        int startIndex = Math.max(lastPointOnPath - 20, 0);

        int endIndex = Math.min(lastPointOnPath + 20, followablePath.size()-1);

        double minDistance = Double.MAX_VALUE;

        List<Vector2D> subList = followablePath.subList(startIndex, endIndex);

        ArrayList<Vector2D> sectionToLook = new ArrayList<>(subList);

        for (Vector2D pos : sectionToLook) {

            double distance = Math.sqrt(
                    Math.pow(robotPos.getX() - pos.getX(), 2) +
                            Math.pow(robotPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                index = followablePath.indexOf(pos);
            }

        }

        lastPointOnPath = index;

        return index;
    }

    public int getClosestPositionOnPathFullPath(Vector2D robotPos) {

        int index = 0;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : followablePath) {

            double distance = Math.sqrt(
                    Math.pow(robotPos.getX() - pos.getX(), 2) +
                            Math.pow(robotPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                index = followablePath.indexOf(pos);
            }
        }

        lastPointOnPath = index;

        return index;
    }

    public Vector2D getErrorToPath(Vector2D robotPos) {

        int index = 0;

        Vector2D error = new Vector2D();

        Vector2D position = new Vector2D();

        double lookaheadDistance;

        int startIndex = Math.max(lastPointOnPath - 20, 0);

        int endIndex = Math.min(lastPointOnPath + 20, followablePath.size());

        int length = endIndex - startIndex + 1;

        double minDistance = Double.MAX_VALUE;

        List<Vector2D> subList = followablePath.subList(startIndex, endIndex + 1);

        ArrayList<Vector2D> sectionToLook = new ArrayList<>(subList);

        for (Vector2D pos : sectionToLook) {

            double distance = Math.sqrt(
                    Math.pow(robotPos.getX() - pos.getX(), 2) +
                            Math.pow(robotPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                index = followablePath.indexOf(pos);
            }

        }

        lastPointOnPath = index;

        lookaheadDistance = Math.abs(Math.hypot(position.getX() - robotPos.getX(), position.getY() - robotPos.getY()));

        index += (int)lookaheadDistance;

        if (index < followablePath.size()-1){
            position = followablePath.get(index);
        }else {
            position = followablePath.get(followablePath.size()-1);
        }

        error.set(position.getX() - robotPos.getX(), position.getY() - robotPos.getY());

        return error;
    }

    public Vector2D getErrorToPath(Vector2D robotPos, int index) {

        Vector2D error = new Vector2D();

        Vector2D position = getPointOnFollowable(index);

        double lookaheadDistance = Math.abs(Math.hypot(position.getX() - robotPos.getX(), position.getY() - robotPos.getY()));

        index += (int)lookaheadDistance;

        if (index < followablePath.size()-1){
            position = followablePath.get(index);
        }else {
            position = followablePath.get(followablePath.size()-1);
        }

        error.set(position.getX() - robotPos.getX(), position.getY() - robotPos.getY());

        return error;
    }


    public PathingVelocity getTargetVelocity(int index){

        PathingVelocity targetVelocity = new PathingVelocity();

        index += 1;

        if (index > pathingVelocity.size()-1){
            targetVelocity.set(0, 0);
        }else {
            targetVelocity.set(pathingVelocity.get(index).getXVelocity(), pathingVelocity.get(index).getYVelocity());
        }

        return targetVelocity;
    }

    public Vector2D getPointOnFollowable(int index){
        return followablePath.get(index);
    }

    public int getLastPoint(){
        int one;

        if (followablePath.size() == 0){
            one = 0;
        }else {
            one = followablePath.size()-1;
        }

        return one;
    }


}
