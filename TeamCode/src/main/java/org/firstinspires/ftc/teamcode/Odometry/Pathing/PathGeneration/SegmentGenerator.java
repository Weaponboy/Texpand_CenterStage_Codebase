package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class SegmentGenerator {

    static Vector2D onTheCurve;

    static ArrayList<Vector2D> Segment = new ArrayList<>();

    static double t = 0.0;

    public ArrayList<Vector2D> copyPath(){
        return Segment;
    }

    public void buildPath(Vector2D startPoint, Vector2D endPoint){
        Segment.clear();
        t = 0;

        do{
            onTheCurve = calculateLine(startPoint, endPoint, t);

            t += 0.001;

            Segment.add(onTheCurve);

        }while (t <= 1.0);

    }

    public void buildPath( Vector2D startPoint, Vector2D controlPoint, Vector2D endPoint){
        Segment.clear();
        t = 0;

        do{
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint, endPoint, t);

            t += 0.001;

            Segment.add(onTheCurve);

        }while (t <= 1.0);

    }

    public void buildPath( Vector2D startPoint, Vector2D controlPoint1, Vector2D controlPoint2, Vector2D endPoint){
        Segment.clear();
        t = 0;

        do{
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint1, controlPoint2, endPoint, t);

            t += 0.001;

            Segment.add(onTheCurve);

        }while (t <= 1.0);

    }

    public void buildPathAvoidance( Vector2D startPoint, Vector2D controlPoint, Vector2D endPoint){

        Segment.clear();

        t = 0;

        do{
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint, endPoint, t);

            t += 0.001;

            Segment.add(onTheCurve);

        }while (t <= 1.0);

    }

    private Vector2D calculateQuadraticBezier(Vector2D start, Vector2D control, Vector2D end, double t) {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;

        double x = uu * start.getX() + 2 * u * t * control.getX() + tt * end.getX();
        double y = uu * start.getY() + 2 * u * t * control.getY() + tt * end.getY();

        return new Vector2D(x, y);
    }

    private Vector2D calculateQuadraticBezier(Vector2D start, Vector2D controlFirst, Vector2D controlSecond, Vector2D end, double t) {
        double u = 1 - t;
        double tt = t * t;
        double ttt = t * t * t;
        double uu = u * u;
        double uuu = u * u * u;

        double x = uuu * start.getX() + 3 * uu * t * controlFirst.getX() + 3 * u * tt * controlSecond.getX() + ttt * end.getX();
        double y = uuu * start.getY() + 3 * uu * t * controlFirst.getY() + 3 * u * tt * controlSecond.getY() + ttt * end.getY();

        return new Vector2D(x, y);
    }

    private Vector2D calculateLine(Vector2D start, Vector2D end, double t) {
        double u = 1 - t;

        double x = start.getX() * u + end.getX() * t;
        double y = start.getY() * u + end.getY() * t;

        return new Vector2D(x, y);
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

    public void blueRightRandomization(){

        Vector2D startPoint = new Vector2D();
        Vector2D endPoint = new Vector2D();
        Vector2D controlPoint = new Vector2D();

        //drop off purple pixel
        startPoint.set(93, 33);
        endPoint.set(95, 85);
        buildPath(startPoint, endPoint);

        //curve to under door
        startPoint.set(endPoint.getX(), endPoint.getY());
        endPoint.set(154, 157);
        controlPoint.set(90, 160);
        buildPath(startPoint, controlPoint, endPoint);

        //curve to backboard
        startPoint.set(endPoint.getX(), endPoint.getY());
        controlPoint.set(302, 154);
        endPoint.set(314, 75);
        buildPath(startPoint, controlPoint, endPoint);

    }

}
