package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Old;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Auto_Control_Points.controlPoints.ePThirdSeg;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.maxYAcceleration;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.velocityDecreasePerPoint;
import static org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry.getMaxVelocity;

import org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Auto_Control_Points.controlPoints;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.SegmentGenerator;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Enums.TargetPoint;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Enums.whatPath;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingVelocity;

import java.util.ArrayList;
import java.util.List;

public class pathBuilder {

    SegmentGenerator segmentGenerator = new SegmentGenerator();

    controlPoints controlPoints = new controlPoints();

    public ArrayList<Vector2D> originalPath = new ArrayList<>();

    public ArrayList<Vector2D> followablePath = new ArrayList<>();

    Vector2D firstPoint = new Vector2D();

    public ArrayList<PathingVelocity> pathingVelocity = new ArrayList<>();

    Vector2D secondPoint = new Vector2D();

    public enum PropPosition{
        first,
        second,
        third
    }

    public enum StartPosition{
        redLeft,
        redRight,
        blueLeft,
        blueRight
    }

    public enum DropPurplePath{
        redLeft,
        redRight,
        blueLeft,
        blueRight
    }

    public enum CollectPath{
        redLeft,
        redRight,
        blueLeft,
        blueRight
    }

    public void buildPath(StartPosition startPosition){

        switch (startPosition) {
            case redLeft:
                dropPurpleRedLeft();
            break;
            case redRight:
                dropPurpleRedRight();
                break;
            case blueLeft:
                dropPurpleBlueLeft();
                break;
            case blueRight:
                dropPurpleBlueRight();
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    public void buildPath(DropPurplePath dropPurplePath){

        switch (dropPurplePath) {
            case redLeft:
                blueRightLongCurve();
                break;
            case redRight:
                redRightToBackBoard();
                break;
            case blueLeft:
                redLeftLongCurve();
                break;
            case blueRight:
                blueLeftLongCurve();
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    public void buildPath(CollectPath collectPath){

        switch (collectPath) {
            case redLeft:
                blueRightLongCurve();
                break;
            case redRight:
                redRightToCollection();
                break;
            case blueLeft:
                redLeftLongCurve();
                break;
            case blueRight:
                blueLeftLongCurve();
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }


    public void buildPath(whatPath path){

        switch (path) {
            case blueRight:
                blueRightLongCurve();
                break;
            case redRight:
                System.out.println("not ready yet");
                break;
            case testCurve:
                dropPurpleBlueRight();
                break;
            case testCurveReverse:
                testCurveReverse();
                break;
            default:
                break;
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    public void buildPath(TargetPoint targetPoint, Vector2D currentPos, Vector2D controlPoint){

        originalPath.clear();
        pathingVelocity.clear();

        switch (targetPoint) {
            case blueBackBoard:
                PathToBackBoard(currentPos, controlPoint, controlPoints.dropAtBlueBackboard);
                break;
            case redBackBoard:
                PathToBackBoard(currentPos, controlPoint, controlPoints.dropAtRedBackboard);
                break;
            case blueCollection:
                PathToCollection(currentPos, controlPoint, controlPoints.collectAtBlue);
                break;
            case redCollection:
                PathToCollection(currentPos, controlPoint, controlPoints.collectAtRed);
                break;
            default:
                break;
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    public void buildPath(TargetPoint targetPoint, Vector2D currentPos){

        originalPath.clear();
        pathingVelocity.clear();

        switch (targetPoint) {
            case blueBackBoard:
                PathToBackBoard(currentPos, controlPoints.dropAtBlueBackboard);
                break;
            case redBackBoard:
                PathToBackBoard(currentPos, controlPoints.dropAtRedBackboard);
                break;
            case blueCollection:
                PathToCollection(currentPos, controlPoints.collectAtBlue);
                break;
            case redCollection:
                PathToCollection(currentPos, controlPoints.collectAtRed);
                break;
            default:
                break;
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    private void buildCurveSegment(Vector2D start, Vector2D control, Vector2D end){
        segmentGenerator.buildPath(start, control, end);
        originalPath.addAll(segmentGenerator.copyPath());
    }

    private void buildLineSegment(Vector2D start, Vector2D end){
        segmentGenerator.buildPath(start, end);
        originalPath.addAll(segmentGenerator.copyPath());
    }

    private int findNumberOfPoints(ArrayList<Vector2D> Path){
        int points;
        double length = calculateTotalDistance(Path);
        points = (int) (length / 0.25);
        return points;
    }

    public void PathToBackBoard(Vector2D startPoint, Vector2D dynamicControlPoint, Vector2D TargetPoint){
        buildCurveSegment(startPoint, dynamicControlPoint, controlPoints.intermediatePointToBackboard);
        buildCurveSegment(controlPoints.intermediatePointToBackboard, controlPoints.intermediateControlToBackboard, TargetPoint);
    }

    public void PathToBackBoard(Vector2D startPoint, Vector2D TargetPoint){
        buildLineSegment(startPoint, controlPoints.intermediatePointToBackboard);
        buildCurveSegment(controlPoints.intermediatePointToBackboard, controlPoints.intermediateControlToBackboard, TargetPoint);
    }

    public void PathToCollection(Vector2D startPoint, Vector2D dynamicControlPoint, Vector2D TargetPoint){
        buildCurveSegment(startPoint, dynamicControlPoint, controlPoints.intermediatePointToCollection);
        buildCurveSegment(controlPoints.intermediatePointToCollection, controlPoints.intermediateControlToCollection, TargetPoint);
    }

    public void PathToCollection(Vector2D startPoint, Vector2D TargetPoint){
        buildLineSegment(startPoint, controlPoints.intermediatePointToCollection);
        buildCurveSegment(controlPoints.intermediatePointToCollection, controlPoints.intermediateControlToCollection, TargetPoint);
    }

    private void testCurve(){
        buildCurveSegment(controlPoints.sPTest, controlPoints.cPTest, controlPoints.ePTest);
    }

    private void testCurveReverse(){
        buildLineSegment(controlPoints.sTest, controlPoints.eTest);
    }

    private Vector2D pathBuilder(ArrayList<Vector2D> originalPath){

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

    private double findAngle(PathingVelocity targetVelocity){

        double magnitude = Math.sqrt(targetVelocity.getXVelocity() * targetVelocity.getXVelocity() + targetVelocity.getYVelocity() * targetVelocity.getYVelocity());

        double radians = Math.atan2(targetVelocity.getYVelocity(), targetVelocity.getXVelocity());

        double degrees = Math.toDegrees(radians);

        if (degrees < 0) {
            degrees += 360;
        }

        return degrees;
    }

    private double motionProfile(){

        PathingVelocity pathVelo;

        double deltaTime;

        double decelerationNumber = 0;

        double pathLength = calculateTotalDistance(followablePath);

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

    private Vector2D findPoint(){
        Vector2D point = new Vector2D();
        return point;
    }

    private double calculateTotalDistance(List<Vector2D> path) {
        double totalDistance = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            Vector2D point1 = path.get(i);
            Vector2D point2 = path.get(i + 1);
            totalDistance += calculateDistance(point1, point2);
        }
        return totalDistance;
    }

    private double calculateDistance(Vector2D point1, Vector2D point2) {
        double dx = point2.getX() - point1.getX();
        double dy = point2.getY() - point1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    private int getClosestPositionOnPath(Vector2D robotPos, ArrayList<Vector2D> path) {

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

    private Vector2D getErrorToPath(Vector2D robotPos, ArrayList<Vector2D> path) {

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

    private PathingVelocity getTargetVelocity(int index){

        PathingVelocity targetVelocity = new PathingVelocity();

        index += 1;

        targetVelocity.set(pathingVelocity.get(index).getXVelocity(), pathingVelocity.get(index).getYVelocity());

        return targetVelocity;
    }

    private Vector2D getPointOnFollowable(int index){
        return followablePath.get(index);
    }


    /**
     * Drop purple pixel path options
     * */

    //done correct positions
    private void dropPurpleRedLeft(){
        buildLineSegment(controlPoints.dropPurpleBlueRightStartPosition, controlPoints.dropPurpleBlueRightEndPosition);
    }

    private void dropPurpleRedRight(){
        buildLineSegment(controlPoints.dropPurpleRedRightStartPosition, controlPoints.dropPurpleRedRightEndPosition);
    }

    private void dropPurpleBlueLeft(){
        buildLineSegment(controlPoints.dropPurpleBlueRightStartPosition, controlPoints.dropPurpleBlueRightEndPosition);
    }

    //done correct Positions
    private void dropPurpleBlueRight(){
        buildLineSegment(controlPoints.dropPurpleBlueRightStartPosition, controlPoints.dropPurpleBlueRightEndPosition);
    }

    /**
     * Drop yellow pixel path options
     **/

    private void blueRightLongCurve(){
        buildCurveSegment(controlPoints.sPSecondSeg, controlPoints.cPSecondSeg, controlPoints.ePSecondSeg);
        buildCurveSegment(controlPoints.sPThirdSeg, controlPoints.cPThirdSeg, ePThirdSeg);
    }

    private void blueLeftLongCurve(){
        buildCurveSegment(controlPoints.sPSecondSeg, controlPoints.cPSecondSeg, controlPoints.ePSecondSeg);
        buildCurveSegment(controlPoints.sPThirdSeg, controlPoints.cPThirdSeg, ePThirdSeg);
    }

    private void redRightToBackBoard(){
        buildLineSegment(controlPoints.dropYellowRedRightStartPosition, controlPoints.dropYellowRedRightEndPosition);
    }

    private void redLeftLongCurve(){
        buildCurveSegment(controlPoints.sPSecondSeg, controlPoints.cPSecondSeg, controlPoints.ePSecondSeg);
        buildCurveSegment(controlPoints.sPThirdSeg, controlPoints.cPThirdSeg, ePThirdSeg);
    }

    /**
     * Collect pixels from stack
     **/

    private void redRightToCollection(){
        buildCurveSegment(controlPoints.driveToCollectionRedRightStartPositionFirstSegment, controlPoints.driveToCollectionRedRightControlPositionFirstSegment, controlPoints.driveToCollectionRedRightEndPositionFirstSegment);
        buildLineSegment(controlPoints.driveToCollectionRedRightStartPositionSecondSegment, controlPoints.driveToCollectionRedRightEndPositionSecondSegment);
    }



}
