package org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class KDTreeExample {

    public Vector2D findClosestPoint(Vector2D queryPoint, List<Vector2D> staticCoordinates) {
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

    public Vector2D findInlineNeighborX(List<Vector2D> staticCoordinates, Vector2D queryPoint, Telemetry telemetry) {

        double minDistance = Double.MAX_VALUE;
        Vector2D nearestNeighbor = null;
        double epsilon = 0.6;

        for (Vector2D point : staticCoordinates) {

            double distance = calculateDistance(point, queryPoint);

            if (Math.abs(point.getY() - queryPoint.getY()) < epsilon && distance < minDistance) {
                minDistance = distance;
                nearestNeighbor = point;
            }
        }

        return nearestNeighbor;
    }

    public Vector2D findInlineNeighborY(List<Vector2D> staticCoordinates, Vector2D queryPoint) {
        double minDistance = Double.MAX_VALUE;
        Vector2D nearestNeighbor = null;
        double epsilon = 0.6;

        for (Vector2D point : staticCoordinates) {

            double distance = calculateDistance(point, queryPoint);

            if (Math.abs(point.getX() - queryPoint.getX()) < epsilon && distance < minDistance) {
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