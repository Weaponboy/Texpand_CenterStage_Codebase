package org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;


public class ObstacleMap {

    public ArrayList<Vector2D> positionList = new ArrayList<>();

    public ArrayList<Vector2D> robotPosition = new ArrayList<>();

    //QuadTree quadTree = new QuadTree(new Rectangle(0, 0, 365, 365)); // Define the bounding box of your field

    public ObstacleMap() {
        SetMap();
    }

    public Vector2D findClosestPosition(Vector2D currentPos) {
        Vector2D closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : positionList) {
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
            for (Vector2D pos2 : positionList) {
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
            for (Vector2D pos2 : positionList) {
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
            for (Vector2D pos2 : positionList) {
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
        Rectangle(117, 0, 183, 5);

        // middle blue truss
        Rectangle(117, 57, 183, 63);

        // door blue truss
        Rectangle(117, 117, 183, 123);

        // door red truss
        Rectangle(117, 241, 183, 247);

        // middle red truss
        Rectangle(117, 301, 183, 307);

        // wall red truss
        Rectangle(117, 360, 183, 365);

    }

    public void Rectangle(double xStart, double yStart, double xEnd, double yEnd){

        double xPosition = xStart;
        double yPosition = yStart;

        for (int i =0; i < ((xEnd - xStart)*2); i++){
            positionList.add(new Vector2D(xPosition, yPosition));
            xPosition += 2;
            if (xPosition == xEnd){
                xPosition = xStart;
                yPosition = yEnd;
            }
        }

        xPosition = xStart;
        yPosition = yStart;

        for (int i =0; i < ((yEnd - yStart)*2); i++){
            positionList.add(new Vector2D(xPosition, yPosition));
            yPosition += 2;
            if (yPosition == yEnd){
                yPosition = yStart;
                xPosition = xEnd;
            }
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
