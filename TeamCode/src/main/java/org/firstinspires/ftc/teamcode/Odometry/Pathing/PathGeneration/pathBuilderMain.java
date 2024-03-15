package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.maxXAcceleration;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.maxYAcceleration;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.velocityDecreasePerPoint;
import static org.firstinspires.ftc.teamcode.hardware._.Odometry.getMaxVelocity;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingVelocity;

import java.util.ArrayList;
import java.util.List;

public class pathBuilderMain {

    public SegmentGenerator segmentGenerator = new SegmentGenerator();

    public ArrayList<Vector2D> originalPath = new ArrayList<>();

    public ArrayList<Vector2D> followablePath = new ArrayList<>();

    public ArrayList<PathingVelocity> pathingVelocity = new ArrayList<>();

    public Vector2D secondPoint = new Vector2D();

    public int findNumberOfPoints(ArrayList<Vector2D> Path){
        int points;
        double length = calculateTotalDistance(Path);
        points = (int) (length / 0.25);
        return points;
    }

    public Vector2D pathBuilder(ArrayList<Vector2D> originalPath){

        Vector2D onTheCurve = new Vector2D();

        followablePath.clear();

        int oldindex = 0;
        int newindex = 0;
        int points = findNumberOfPoints(originalPath) - 1;

        double XFirst = originalPath.get(oldindex).getX();
        double YFirst = originalPath.get(oldindex).getY();

        oldindex += 6;

        secondPoint = originalPath.get(oldindex);

        double xChange;
        double hypotenuse;

        while (oldindex != originalPath.size()){

            secondPoint = originalPath.get(oldindex);

            //assumes that the curve goes in the positive direction
            hypotenuse = Math.hypot(secondPoint.getX() - XFirst, secondPoint.getY() - YFirst);

            xChange = (secondPoint.getX() - XFirst);
            double yChange = (secondPoint.getY() - YFirst);

            double angle = 0;

            angle = Math.atan2(yChange, xChange);

            double newY = 0.25 * Math.sin(angle);
            double newX = 0.25 * Math.cos(angle);

            if (hypotenuse > 0.25){

                XFirst += newX;
                YFirst += newY;

                onTheCurve = new Vector2D(XFirst , YFirst);

                followablePath.add(onTheCurve);

            }else{
                oldindex++;
            }
        }

        return onTheCurve;

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

    public double motionProfile(){

        PathingVelocity pathVelo;

        double deltaTime;

        int decelerationNumber = 0;

        double pathLength = calculateTotalDistance(followablePath);

        double acceleration_dt = (double) getMaxVelocity() / maxXAcceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = pathLength / 1.2;

        double acceleration_distance;

        acceleration_distance = 0.5 * maxXAcceleration * acceleration_dt * 2;

        if (acceleration_distance > halfway_distance){
            acceleration_dt = (halfway_distance / maxXAcceleration);
        }

        acceleration_distance = 0.5 * maxXAcceleration * acceleration_dt * 2;

        double max_velocity = maxXAcceleration * acceleration_dt;

        double deceleration_dt = acceleration_distance;

        int decIndex = (int) (deceleration_dt/0.25);

        System.out.println(acceleration_distance);
        System.out.println(acceleration_dt);
        System.out.println(max_velocity);
        System.out.println(decIndex);
        System.out.println(pathLength);

        int range;

//        if(decIndex > halfway_distance){
//            decIndex = (int) halfway_distance;
//        }

        for (int i = 0; i < followablePath.size() - 1; i++) {

            if (i + decIndex >= followablePath.size()){

                decelerationNumber += 1;

                range = Math.abs(decIndex - decelerationNumber);

                double DecSlope = (double) range / (double) Math.abs(decIndex) * 100;

                DecSlope = DecSlope*0.01;

                Vector2D currentPoint = followablePath.get(i);
                Vector2D nextPoint = followablePath.get(i + 1);

                double deltaX = nextPoint.getX() - currentPoint.getX();
                double deltaY = nextPoint.getY() - currentPoint.getY();

                double percentSum = (Math.abs(deltaX)/0.25)+(Math.abs(deltaY)/0.25);

                double Xfactor = (deltaX/0.25) * (1/percentSum);
                double Yfactor = (deltaY/0.25) * (1/percentSum);

                double velocityXValue = (Xfactor * max_velocity) * DecSlope;
                double velocityYValue = (Yfactor * max_velocity) * DecSlope;

                System.out.println("Vector dec " + (Math.abs(velocityYValue)+Math.abs(velocityXValue)));

                pathVelo = new PathingVelocity(velocityXValue,velocityYValue);

                pathingVelocity.add(pathVelo);

            }else {

                Vector2D currentPoint = followablePath.get(i);
                Vector2D nextPoint = followablePath.get(i + 1);

                double deltaX = nextPoint.getX() - currentPoint.getX();
                double deltaY = nextPoint.getY() - currentPoint.getY();

                double percentSum = (Math.abs(deltaX)/0.25)+(Math.abs(deltaY)/0.25);

                double Xfactor = (deltaX/0.25) * (1/percentSum);
                double Yfactor = (deltaY/0.25) * (1/percentSum);

                double velocityXValue = (Xfactor) * max_velocity;
                double velocityYValue = (Yfactor) * max_velocity;

                System.out.println("Vector " + (Math.abs(velocityYValue)+Math.abs(velocityXValue)));

                pathVelo = new PathingVelocity(velocityXValue, velocityYValue);

                pathingVelocity.add(pathVelo);

            }

        }

        return 0;

    }

    public double calculateTotalDistance(List<Vector2D> path) {
        double totalDistance = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            Vector2D point1 = path.get(i);
            Vector2D point2 = path.get(i + 1);
            totalDistance += calculateDistance(point1, point2);
        }
        return totalDistance;
    }

    public double calculateDistance(Vector2D point1, Vector2D point2) {
        double dx = point2.getX() - point1.getX();
        double dy = point2.getY() - point1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public int getClosestPositionOnPath(Vector2D robotPos, ArrayList<Vector2D> path) {

        int index = 0;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : path) {
            double distance = Math.sqrt(
                    Math.pow(robotPos.getX() - pos.getX(), 2) +
                            Math.pow(robotPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                index = path.indexOf(pos);
            }
        }

        return index;
    }

    public Vector2D getErrorToPath(Vector2D robotPos, ArrayList<Vector2D> path) {

        int index = 0;

        Vector2D error = new Vector2D();

        Vector2D position = new Vector2D();

        double lookaheadDistance;

        double incrementDistance = 1;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : path) {
            double distance = Math.sqrt(
                    Math.pow(robotPos.getX() - pos.getX(), 2) +
                            Math.pow(robotPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                index = path.indexOf(pos);
                position.set(pos.getX(), pos.getY());
            }
        }

        lookaheadDistance = Math.abs(Math.hypot(position.getX() - robotPos.getX(), position.getY() - robotPos.getY()));

        index += (int)lookaheadDistance;

        position = path.get(index);

        error.set(position.getX() - robotPos.getX(), position.getY() - robotPos.getY());

        return error;
    }

    public PathingVelocity getTargetVelocity(int index){

        PathingVelocity targetVelocity = new PathingVelocity();

        index += 1;

        targetVelocity.set(pathingVelocity.get(index).getXVelocity(), pathingVelocity.get(index).getYVelocity());

        return targetVelocity;
    }

    public Vector2D getPointOnFollowable(int index){
        return followablePath.get(index);
    }

    public void buildCurveSegment(Vector2D start, Vector2D control, Vector2D end){
        segmentGenerator.buildPath(start, control, end);
        originalPath.addAll(segmentGenerator.copyPath());
    }

    public void buildCurveSegment(Vector2D start, Vector2D control1, Vector2D control2, Vector2D end){
        segmentGenerator.buildPath(start, control1, control2, end);
        originalPath.addAll(segmentGenerator.copyPath());
    }

    public void buildLineSegment(Vector2D start, Vector2D end){
        segmentGenerator.buildPath(start, end);
        originalPath.addAll(segmentGenerator.copyPath());
    }

    public void clearAll(){
        followablePath.clear();
        originalPath.clear();
        pathingVelocity.clear();
    }

}
