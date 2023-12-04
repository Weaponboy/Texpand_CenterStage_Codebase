package org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance;

import java.util.ArrayList;

public class buildRobotBoundary {

    public ArrayList<Vector2D> xTop = new ArrayList<>();
    public ArrayList<Vector2D> xbottom = new ArrayList<>();
    public ArrayList<Vector2D> yRight = new ArrayList<>();
    public ArrayList<Vector2D> yLeft = new ArrayList<>();

    public Vector2D robotPos = new Vector2D();

    public Vector2D frontRightCorner = new Vector2D();
    public Vector2D frontLeftCorner = new Vector2D();
    public Vector2D backRightCorner = new Vector2D();
    public Vector2D backLeftCorner = new Vector2D();

    public buildRobotBoundary(){}

    public enum Side{
        first,
        second,
        third,
        forth
    }

    public void buildRobotPosition(Vector2D robotCenterPos, double heading){

        double robotOffsetToLeft = -21;
        double robotOffsetToRight = 21;
        double robotOffsetToBack = -22;
        double robotOffsetToFront = 22;


        frontRightCorner.setX(robotCenterPos.getX() + (robotOffsetToFront * Math.cos(Math.toRadians(heading)) - robotOffsetToRight * Math.sin(Math.toRadians(heading))));
        frontRightCorner.setY(robotCenterPos.getY() + (robotOffsetToFront * Math.sin(Math.toRadians(heading)) + robotOffsetToRight * Math.cos(Math.toRadians(heading))));

        frontLeftCorner.setX(robotCenterPos.getX() + (robotOffsetToFront * Math.cos(Math.toRadians(heading)) - robotOffsetToLeft * Math.sin(Math.toRadians(heading))));
        frontLeftCorner.setY(robotCenterPos.getY() + (robotOffsetToFront * Math.sin(Math.toRadians(heading)) + robotOffsetToLeft * Math.cos(Math.toRadians(heading))));

        backRightCorner.setX(robotCenterPos.getX() + (robotOffsetToBack * Math.cos(Math.toRadians(heading)) - robotOffsetToRight * Math.sin(Math.toRadians(heading))));
        backRightCorner.setY(robotCenterPos.getY() + (robotOffsetToBack * Math.sin(Math.toRadians(heading)) + robotOffsetToRight * Math.cos(Math.toRadians(heading))));

        backLeftCorner.setX(robotCenterPos.getX() + (robotOffsetToBack * Math.cos(Math.toRadians(heading)) - robotOffsetToLeft * Math.sin(Math.toRadians(heading))));
        backLeftCorner.setY(robotCenterPos.getY() + (robotOffsetToBack * Math.sin(Math.toRadians(heading)) + robotOffsetToLeft * Math.cos(Math.toRadians(heading))));


        buildRobot(frontRightCorner.getX(), frontRightCorner.getY(), backRightCorner.getX(), backRightCorner.getY(), backLeftCorner.getX(), backLeftCorner.getY(), frontLeftCorner.getX(), frontLeftCorner.getY());

    }

    public void buildRobot(double corner1_x, double corner1_y, double corner2_x, double corner2_y, double corner3_x, double corner3_y, double corner4_x, double corner4_y) {

        Side side = Side.first;

        xTop.clear();

        Vector2D PointToAdd = new Vector2D();

        double XFirst;
        double YFirst;

        double changeInY;
        double changeInX;

        double angle;

        double newY;
        double newX;

        boolean exitLoop = false;

        do {
            switch (side){
                case first:

                    XFirst = corner1_x;
                    YFirst = corner1_y;

                    changeInY = corner2_y - corner1_y;
                    changeInX = corner2_x - corner1_x;

                    angle = Math.atan2(changeInY, changeInX);

                    newY = 1 * Math.sin(angle);
                    newX = 1 * Math.cos(angle);

                    for (int i = 0; i < (int) Math.abs(Math.hypot(changeInX, changeInY)-1) ; i++) {


                        YFirst += newY;
                        XFirst += newX;

                        PointToAdd = new Vector2D(XFirst, YFirst);

                        yRight.add(PointToAdd);

                    }

                    side = Side.second;

                    break;
                case second:

                    XFirst = corner2_x;
                    YFirst = corner2_y;

                    changeInY = corner3_y - corner2_y;
                    changeInX = corner3_x - corner2_x;

                    angle = Math.atan2(changeInY, changeInX);

                    newY = 1 * Math.sin(angle);
                    newX = 1 * Math.cos(angle);

                    for (int i = 0; i < (int) Math.abs(Math.hypot(changeInX, changeInY)-1) ; i++) {

                        XFirst += newX;
                        YFirst += newY;

                        PointToAdd = new Vector2D(XFirst, YFirst);

                        xbottom.add(PointToAdd);

                    }

                    side = Side.third;
                    break;
                case third:
                    XFirst = corner3_x;
                    YFirst = corner3_y;

                    changeInY = corner4_y - corner3_y;
                    changeInX = corner4_x - corner3_x;

                    angle = Math.atan2(changeInY, changeInX);

                    newY = 1 * Math.sin(angle);
                    newX = 1 * Math.cos(angle);

                    for (int i = 0; i < (int) Math.abs(Math.hypot(changeInX, changeInY)-1) ; i++) {

                        XFirst += newX;
                        YFirst += newY;

                        PointToAdd = new Vector2D(XFirst, YFirst);

                        yLeft.add(PointToAdd);

                    }

                    side = Side.forth;

                    break;
                case forth:

                    XFirst = corner4_x;
                    YFirst = corner4_y;

                    changeInY = corner1_y - corner4_y;
                    changeInX = corner1_x - corner4_x;

                    angle = Math.atan2(changeInY, changeInX);

                    newY = 1 * Math.sin(angle);
                    newX = 1 * Math.cos(angle);

                    for (int i = 0; i < (int) Math.abs(Math.hypot(changeInX, changeInY)-1) ; i++) {

                        XFirst += newX;
                        YFirst += newY;

                        PointToAdd = new Vector2D(XFirst, YFirst);

                        xTop.add(PointToAdd);

                    }

                    exitLoop = true;

                    break;
                default:

            }
        }while (!exitLoop);

    }

}
