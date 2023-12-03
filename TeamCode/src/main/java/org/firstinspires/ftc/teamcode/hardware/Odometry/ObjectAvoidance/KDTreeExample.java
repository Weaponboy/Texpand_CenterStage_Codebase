package org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance;

import java.util.ArrayList;
import java.util.List;

public class KDTreeExample {

    public Vector2D buildRobot(Vector2D queryPoint, List<Vector2D> staticCoordinates) {
        return findNearestNeighbor(staticCoordinates, queryPoint);
    }

    public static Vector2D findNearestNeighbor(List<Vector2D> staticCoordinates, Vector2D queryPoint) {
        double minDistance = Double.MAX_VALUE;
        Vector2D nearestNeighbor = null;

        for (Vector2D point : staticCoordinates) {
            double distance = calculateDistance(point, queryPoint);

            if (distance < minDistance) {
                minDistance = distance;
                nearestNeighbor = point;
            }
        }

        return nearestNeighbor;
    }

    private static double calculateDistance(Vector2D point1, Vector2D point2) {
        double dx = point1.getX() - point2.getX();
        double dy = point1.getY() - point2.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}