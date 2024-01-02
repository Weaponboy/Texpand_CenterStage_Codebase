package org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.scaleFactor;

import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingPower;

import java.util.ArrayList;


public class ObstacleMap {

    public ArrayList<Vector2D> realObstacles = new ArrayList<>();

    public ArrayList<Vector2D> InnerRing = new ArrayList<>();

    public ArrayList<PathingPower> InnerRingVectors = new ArrayList<>();

    public ArrayList<Vector2D> oneObstacle = new ArrayList<>();
    public ArrayList<Vector2D> oneObstacleOuter = new ArrayList<>();

    public ArrayList<Vector2D> robotPosition = new ArrayList<>();

    //QuadTree quadTree = new QuadTree(new Rectangle(0, 0, 365, 365)); // Define the bounding box of your field

    public ObstacleMap() {
        SetMap();
    }

    public Vector2D findClosestPosition(Vector2D currentPos) {
        Vector2D closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : realObstacles) {
            double distance = Math.sqrt(
                    Math.pow(currentPos.getX() - pos.getX(), 2) +
                            Math.pow(currentPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                closest = pos;
            }
        }

        return closest;
    }

    public Vector2D findClosestPositionInner(Vector2D currentPos) {
        Vector2D closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : oneObstacle) {
            double distance = Math.sqrt(
                    Math.pow(currentPos.getX() - pos.getX(), 2) +
                            Math.pow(currentPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                closest = pos;
            }
        }

        return closest;
    }

    public Vector2D findClosestPositions(ArrayList<Vector2D> robotPosition) {

        Vector2D closest1 = null;
        Vector2D closest2 = null;

        double distanceToOb = 0;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos1 : robotPosition) {
            for (Vector2D pos2 : realObstacles) {
                double distance = Math.abs(pos1.getX() - pos2.getX());

                if (distance < minDistance) {
                    minDistance = distance;
                    closest1 = pos1;
                    closest2 = pos2;
                    distanceToOb = distance;
                }


            }
        }

        return closest2;
    }

    public void buildRobotPosition(Vector2D robotCenterPos, double heading){

        double robotOffsetToLeft = -21;
        double robotOffsetToRight = 21;
        double robotOffsetToBack = -22;
        double robotOffsetToFront = 22;

        Vector2D frontRightCorner = new Vector2D();
        Vector2D frontLeftCorner = new Vector2D();
        Vector2D backRightCorner = new Vector2D();
        Vector2D backLeftCorner = new Vector2D();

        frontRightCorner.setX(robotCenterPos.getX() + (robotOffsetToFront * Math.cos(Math.toRadians(heading)) - robotOffsetToRight * Math.sin(Math.toRadians(heading))));
        frontRightCorner.setY(robotCenterPos.getY() + (robotOffsetToFront * Math.sin(Math.toRadians(heading)) + robotOffsetToRight * Math.cos(Math.toRadians(heading))));

        frontLeftCorner.setX(robotCenterPos.getX() + (robotOffsetToFront * Math.cos(Math.toRadians(heading)) - robotOffsetToLeft * Math.sin(Math.toRadians(heading))));
        frontLeftCorner.setY(robotCenterPos.getY() + (robotOffsetToFront * Math.sin(Math.toRadians(heading)) + robotOffsetToLeft * Math.cos(Math.toRadians(heading))));

        backRightCorner.setX(robotCenterPos.getX() + (robotOffsetToBack * Math.cos(Math.toRadians(heading)) - robotOffsetToRight * Math.sin(Math.toRadians(heading))));
        backRightCorner.setY(robotCenterPos.getY() + (robotOffsetToBack * Math.sin(Math.toRadians(heading)) + robotOffsetToRight * Math.cos(Math.toRadians(heading))));

        backLeftCorner.setX(robotCenterPos.getX() + (robotOffsetToBack * Math.cos(Math.toRadians(heading)) - robotOffsetToLeft * Math.sin(Math.toRadians(heading))));
        backLeftCorner.setY(robotCenterPos.getY() + (robotOffsetToBack * Math.sin(Math.toRadians(heading)) + robotOffsetToLeft * Math.cos(Math.toRadians(heading))));

        buildRobot(frontRightCorner.getX(), frontRightCorner.getY(), frontLeftCorner.getX(), frontLeftCorner.getY(), backRightCorner.getX(), backRightCorner.getY(), backLeftCorner.getX(), backLeftCorner.getY());

    }

    public Vector2D findClosestXPosition(ArrayList<Vector2D> robotPosition) {

        Vector2D closest1 = null;
        Vector2D closest2 = null;

        double distanceToOb = 0;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos1 : robotPosition) {
            for (Vector2D pos2 : realObstacles) {
                double distance = Math.abs(pos1.getX() - pos2.getX());

                if (distance < minDistance) {
                    minDistance = distance;
                    closest2 = new Vector2D(pos1.getX(), pos2.getX());
                }


            }
        }

        return closest2;
    }

    public Vector2D findClosestYPosition(ArrayList<Vector2D> robotPosition) {

        Vector2D closest1 = null;
        Vector2D closest2 = null;

        double distanceToOb = 0;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos1 : robotPosition) {
            for (Vector2D pos2 : realObstacles) {
                double distance = Math.abs(pos1.getY() - pos2.getY());

                if (distance < minDistance) {
                    minDistance = distance;
                    closest2 = new Vector2D(pos1.getY(), pos2.getY());
                }


            }
        }

        return closest2;
    }

    public void SetMap(){

        // wall blue truss
        Rectangle(118, 0, 182, 5);

        // middle blue truss
        Rectangle(118, 57, 182, 63);

        // door blue truss
        Rectangle(118, 117, 182, 123);

        // door red truss
        Rectangle(118, 241, 182, 247);

        // middle red truss
        Rectangle(118, 301, 182, 307);

        // wall red truss
        Rectangle(118, 360, 182, 365);

    }

    public void SetMapVectorBased(){

        // wall blue truss
        RectangleOne(118, 0, 182, 4);
        RectangleInnerRing(97, 0, 203, 25);

        // middle blue truss
        RectangleOne(118, 58, 182, 62);
        RectangleInnerRing(97, 37, 203, 83);

        // door blue truss
        RectangleOne(118, 118, 182, 122);
        RectangleInnerRing(97, 97, 203, 143);

        // door red truss
        RectangleOne(118, 238, 182, 242);
        RectangleInnerRing(118, 217, 182, 221);

        // middle red truss
        RectangleOne(118, 298, 182, 302);
        RectangleInnerRing(118, 277, 182, 323);

        // wall red truss
        RectangleOne(118, 356, 182, 360);
        RectangleInnerRing(118, 335, 182, 360);

    }

    public void Rectangle(double xStart, double yStart, double xEnd, double yEnd){

        double xPosition = xStart;
        double yPosition = yStart;

        for (int i =0; i < ((xEnd - xStart)*2); i++){
            realObstacles.add(new Vector2D(xPosition, yPosition));
            xPosition += 2;
            if (xPosition == xEnd){
                xPosition = xStart;
                yPosition = yEnd;
            }
        }

        xPosition = xStart;
        yPosition = yStart;

        for (int i =0; i < ((yEnd - yStart)*2); i++){
            realObstacles.add(new Vector2D(xPosition, yPosition));
            yPosition += 2;
            if (yPosition == yEnd){
                yPosition = yStart;
                xPosition = xEnd;
            }
        }

    }

    public void RectangleOne(double xStart, double yStart, double xEnd, double yEnd){

        oneObstacle.clear();

        double xPosition = xStart;
        double yPosition = yStart;

        for (int i =0; i < ((xEnd - xStart)*2); i++){
            oneObstacle.add(new Vector2D(xPosition, yPosition));
            xPosition += 2;
            if (xPosition == xEnd){
                xPosition = xStart;
                yPosition = yEnd;
            }
        }

        xPosition = xStart;
        yPosition = yStart;

        for (int i =0; i < ((yEnd - yStart)*2); i++){
            oneObstacle.add(new Vector2D(xPosition, yPosition));
            yPosition += 2;
            if (yPosition == yEnd){
                yPosition = yStart;
                xPosition = xEnd;
            }
        }

    }

    public void RectangleInnerRing(double xStart, double yStart, double xEnd, double yEnd){

        double xPosition = xStart;
        double yPosition = yStart;

        for (int i =0; i < ((xEnd - xStart)*2); i++){
            oneObstacleOuter.add(new Vector2D(xPosition, yPosition));
            xPosition += 2;
            if (xPosition == xEnd){
                xPosition = xStart;
                yPosition = yEnd;
            }
        }

        xPosition = xStart;
        yPosition = yStart;

        for (int i =0; i < ((yEnd - yStart)*2); i++){
            oneObstacleOuter.add(new Vector2D(xPosition, yPosition));
            yPosition += 2;
            if (yPosition == yEnd){
                yPosition = yStart;
                xPosition = xEnd;
            }
        }

    }

    public void findVectors(){

        Vector2D positionToAdd;
        PathingPower powerToAdd;

        for (int i = 0; i < oneObstacleOuter.size(); i++){

            Vector2D currentPos = oneObstacleOuter.get(i);

            Vector2D closestInside = findClosestPositionInner(currentPos);

            double dx = currentPos.getX() - closestInside.getX();
            double dy = currentPos.getY() - closestInside.getY();

            if (dy == 0){
                double dxfactor = 1/dx;
                powerToAdd = new PathingPower((dxfactor*dx)*scaleFactor, 0);
            } else if (dx == 0) {

            } else {

            }

            double factor = 1/dx;

        }

    }

    public enum Side{
        first,
        second,
        third,
        forth
    }

    public void buildRobot(double corner1_x, double corner1_y, double corner2_x, double corner2_y, double corner3_x, double corner3_y, double corner4_x, double corner4_y) {

        Side side = Side.first;

        robotPosition.clear();

        Vector2D PointToAdd = new Vector2D();

        double XFirst;
        double YFirst;

        double changeInY;
        double changeInX;

        double angle;

        double newY;
        double newX;

        do {
            switch (side){
                case first:

                    XFirst = corner1_x;
                    YFirst = corner1_y;

                    changeInY = corner1_y - corner2_y;
                    changeInX = corner1_x - corner2_x;

                    angle = Math.atan2(changeInY, changeInX);

                    newY = 1 * Math.sin(angle);
                    newX = 1 * Math.cos(angle);

                    for (int i = 0; i < (int) (Math.hypot(changeInX, changeInY)-1) ; i++) {

                        XFirst += newX;
                        YFirst += newY;

                        PointToAdd = new Vector2D(XFirst, YFirst);

                        robotPosition.add(PointToAdd);

                    }

                    break;
                case second:

                    XFirst = corner2_x;
                    YFirst = corner2_y;

                    changeInY = corner2_y - corner3_y;
                    changeInX = corner2_x - corner3_x;

                    angle = Math.atan2(changeInY, changeInX);

                    newY = 1 * Math.sin(angle);
                    newX = 1 * Math.cos(angle);

                    for (int i = 0; i < (int) (Math.hypot(changeInX, changeInY)-1) ; i++) {

                        XFirst += newX;
                        YFirst += newY;

                        PointToAdd = new Vector2D(XFirst, YFirst);

                        robotPosition.add(PointToAdd);

                    }

                    break;
                case third:

                    XFirst = corner3_x;
                    YFirst = corner3_y;

                    changeInY = corner3_y - corner4_y;
                    changeInX = corner3_x - corner4_x;

                    angle = Math.atan2(changeInY, changeInX);

                    newY = 1 * Math.sin(angle);
                    newX = 1 * Math.cos(angle);

                    for (int i = 0; i < (int) (Math.hypot(changeInX, changeInY)-1) ; i++) {

                        XFirst += newX;
                        YFirst += newY;

                        PointToAdd = new Vector2D(XFirst, YFirst);

                        robotPosition.add(PointToAdd);

                    }

                    break;
                case forth:

                    XFirst = corner4_x;
                    YFirst = corner4_y;

                    changeInY = corner4_y - corner1_y;
                    changeInX = corner4_x - corner1_x;

                    angle = Math.atan2(changeInY, changeInX);

                    newY = 1 * Math.sin(angle);
                    newX = 1 * Math.cos(angle);

                    for (int i = 0; i < (int) (Math.hypot(changeInX, changeInY)-1) ; i++) {

                        XFirst += newX;
                        YFirst += newY;

                        PointToAdd = new Vector2D(XFirst, YFirst);

                        robotPosition.add(PointToAdd);

                    }

                    break;
                default:

            }
        }while (Math.abs(PointToAdd.getX() - corner4_x) > 0.5 && Math.abs(PointToAdd.getY() - corner4_y) > 0.5);

    }

}
