package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;
import static org.firstinspires.ftc.teamcode.Teleop.CompitionTeleops.RedTeleop.targetPoint;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;

import java.util.ArrayList;

public interface TeleopPathing {

    Vector2D threeLeftRed = new Vector2D(getRealCoords(305), getRealCoords(250));
    Vector2D twoLeftRed = new Vector2D(getRealCoords(305), getRealCoords(255));
    Vector2D oneLeftRed = new Vector2D(getRealCoords(305), getRealCoords(262.5));
    Vector2D middleRed = new Vector2D(getRealCoords(305), getRealCoords(270));
    Vector2D oneRightRed = new Vector2D(getRealCoords(305), getRealCoords(277.5));
    Vector2D twoRightRed = new Vector2D(getRealCoords(305), getRealCoords(285));
    Vector2D threeRightRed = new Vector2D(getRealCoords(305), getRealCoords(290));


    Vector2D threeLeftBlue = new Vector2D(getRealCoords(305), getRealCoords(70));
    Vector2D twoLeftBlue = new Vector2D(getRealCoords(305), getRealCoords(75));
    Vector2D oneLeftBlue = new Vector2D(getRealCoords(305), getRealCoords(82.5));
    Vector2D middleBlue = new Vector2D(getRealCoords(305), getRealCoords(90));
    Vector2D oneRightBlue = new Vector2D(getRealCoords(305), getRealCoords(97.5));
    Vector2D twoRightBlue = new Vector2D(getRealCoords(305), getRealCoords(105));
    Vector2D threeRightBlue = new Vector2D(getRealCoords(305), getRealCoords(110));


    default Vector2D findClosestPosBlue(Vector2D robotPos){

        ArrayList<Vector2D> sectionToLook = new ArrayList<>();

        sectionToLook.add(threeLeftBlue);
        sectionToLook.add(twoLeftBlue);
        sectionToLook.add(oneLeftBlue);
        sectionToLook.add(middleBlue);
        sectionToLook.add(oneRightBlue);
        sectionToLook.add(twoRightBlue);
        sectionToLook.add(threeRightBlue);

        int index = 0;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : sectionToLook) {

            double distance = Math.sqrt(
                    Math.pow(robotPos.getX() - pos.getX(), 2) +
                            Math.pow(robotPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                index = sectionToLook.indexOf(pos);
            }

        }

        return sectionToLook.get(index);
    }

    default int findClosestPosRed(Vector2D robotPos){

        ArrayList<Vector2D> sectionToLook = new ArrayList<>();

        sectionToLook.add(threeLeftRed);
        sectionToLook.add(twoLeftRed);
        sectionToLook.add(oneLeftRed);
        sectionToLook.add(middleRed);
        sectionToLook.add(oneRightRed);
        sectionToLook.add(twoRightRed);
        sectionToLook.add(threeRightRed);

        int index = 0;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : sectionToLook) {

            double distance = Math.sqrt(
                    Math.pow(robotPos.getX() - pos.getX(), 2) +
                            Math.pow(robotPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                index = sectionToLook.indexOf(pos);
                targetPoint.set(pos.getX(), pos.getY());
            }

        }

        return index+1;
    }

}
