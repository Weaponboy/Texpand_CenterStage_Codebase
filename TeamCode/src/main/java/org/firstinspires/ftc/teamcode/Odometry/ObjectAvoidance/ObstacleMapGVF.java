package org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingPower;

import java.util.ArrayList;


public class ObstacleMapGVF {

    public ArrayList<Vector2D> VectorField = new ArrayList<>();
    public ArrayList<PathingPower> VectorPowers = new ArrayList<>();

    public ArrayList<Vector2D> realObstacle = new ArrayList<>();

    public ArrayList<Vector2D> VectorRing = new ArrayList<>();

    public ObstacleMapGVF() {
        SetMap();
    }

    public Vector2D findClosestPosition(Vector2D currentPos) {
        Vector2D closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : VectorField) {
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

        for (Vector2D pos : realObstacle) {
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

    public void SetMap(){

        // wall blue truss
        RealRectangle(118, 0, 182, 4);
        VectorRect(97, 0, 203, 25);

        findVectors();

        // middle blue truss
        RealRectangle(118, 58, 182, 62);
        VectorRect(97, 37, 203, 83);

        findVectors();

        // door blue truss
        RealRectangle(118, 118, 182, 122);
        VectorRect(97, 97, 203, 143);

        findVectors();

        // door red truss
        RealRectangle(118, 238, 182, 242);
        VectorRect(118, 217, 182, 221);

        findVectors();

        // middle red truss
        RealRectangle(118, 298, 182, 302);
        VectorRect(118, 277, 182, 323);

        findVectors();

        // wall red truss
        RealRectangle(118, 356, 182, 360);
        VectorRect(118, 335, 182, 360);

        findVectors();

    }

    public void RealRectangle(double xStart, double yStart, double xEnd, double yEnd){

        realObstacle.clear();

        double xPosition = xStart;
        double yPosition = yStart;

        for (int i =0; i < ((xEnd - xStart)*2); i++){
            realObstacle.add(new Vector2D(xPosition, yPosition));
            xPosition += 2;
            if (xPosition == xEnd){
                xPosition = xStart;
                yPosition = yEnd;
            }
        }

        xPosition = xStart;
        yPosition = yStart;

        for (int i =0; i < ((yEnd - yStart)*2); i++){
            realObstacle.add(new Vector2D(xPosition, yPosition));
            yPosition += 2;
            if (yPosition == yEnd){
                yPosition = yStart;
                xPosition = xEnd;
            }
        }

    }

    public void VectorRect(double xStart, double yStart, double xEnd, double yEnd){

        double xPosition = xStart;
        double yPosition = yStart;

        for (int i =0; i < ((xEnd - xStart)*2); i++){
            VectorRing.add(new Vector2D(xPosition, yPosition));
            xPosition += 2;
            if (xPosition == xEnd){
                xPosition = xStart;
                yPosition = yEnd;
            }
        }

        xPosition = xStart;
        yPosition = yStart;

        for (int i =0; i < ((yEnd - yStart)*2); i++){
            VectorRing.add(new Vector2D(xPosition, yPosition));
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

        for (int i = 0; i < VectorRing.size(); i++){

            Vector2D currentPos = VectorRing.get(i);

            Vector2D closestInside = findClosestPositionInner(currentPos);

            double dx = currentPos.getX() - closestInside.getX();
            double dy = currentPos.getY() - closestInside.getY();

            double xVelo = 0;
            double yVelo = 0;

            if (Math.abs(dx) < 1){
                xVelo = 0;
            }else if (dx < 0){
                xVelo = -65;
            }else if (dx > 0){
                xVelo = 65;
            }

            if (Math.abs(dy) < 1) {
                yVelo = 0;
            }else if (dy < 0){
                yVelo = -43;
            }else if (dy > 0){
                yVelo = 43;
            }

            powerToAdd = new PathingPower(xVelo, yVelo);
            positionToAdd = closestInside;

            realObstacle.add(positionToAdd);
            VectorPowers.add(powerToAdd);

        }

    }

}
