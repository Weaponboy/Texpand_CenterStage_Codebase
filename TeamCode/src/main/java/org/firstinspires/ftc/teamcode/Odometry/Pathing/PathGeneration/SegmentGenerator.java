package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;

import java.util.ArrayList;

public class SegmentGenerator {

    static Vector2D onTheCurve;

    static ArrayList<Vector2D> Segment = new ArrayList<>();

    static ArrayList<Double> curvaturePoints = new ArrayList<>();

    static double t = 0.0;

    public ArrayList<Vector2D> copyPath(){
        return Segment;
    }

    public ArrayList<Double> copyCurvature(){
        return curvaturePoints;
    }

    /**Straight line gen method*/
    public void buildPath(Vector2D startPoint, Vector2D endPoint){

        //Clear segment array of the last segment
        Segment.clear();

        //reset t back to zero
        t = 0;

        //Repeat while t is less than 1
        do{
            //call method with equations, pass it the start and end point. as well as t
            onTheCurve = calculateLine(startPoint, endPoint, t);

            //increment t slightly, i prefer having this very low just in increase accuracy
            t += 0.001;

            //add the calculated point to the array
            Segment.add(onTheCurve);

        }while (t <= 1.0);

    }

    /**Three point curve gen method*/
    public void buildPath( Vector2D startPoint, Vector2D controlPoint, Vector2D endPoint){

        //Clear segment array of the last segment
        Segment.clear();

        //reset t back to zero
        t = 0;

        //Repeat while t is less than 1
        do{
            //call method with equations, pass it the start, control and end point. as well as t
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint, endPoint, t);

            //increment t slightly, i prefer having this very low just to increase accuracy
            t += 0.001;

            //add the calculated point to the array
            Segment.add(onTheCurve);

        }while (t <= 1.0);

    }

    /**Four point curve gen method*/
    public void buildPath( Vector2D startPoint, Vector2D controlPoint1, Vector2D controlPoint2, Vector2D endPoint){

        //Clear segment array of the last segment
        Segment.clear();

        //reset t back to zero
        t = 0;

        //Repeat while t is less than 1
        do{
            //call method with equations, pass it the start, control and end point. as well as t
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint1, controlPoint2, endPoint, t);

            //increment t slightly, i prefer having this very low just in increase accuracy
            t += 0.001;

            //add the calculated point to the array
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

    /**Three point curve gen equations method*/
    private Vector2D calculateQuadraticBezier(Vector2D start, Vector2D control, Vector2D end, double t) {
        //Define t constants
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;

        //calculate X and Y coordinates by incrementing t between the points
        double x = uu * start.getX() + 2 * u * t * control.getX() + tt * end.getX();
        double y = uu * start.getY() + 2 * u * t * control.getY() + tt * end.getY();

        //return X and Y in a custom position class
        return new Vector2D(x, y);
    }

    /**Four point curve gen equations method*/
    private Vector2D calculateQuadraticBezier(Vector2D start, Vector2D controlFirst, Vector2D controlSecond, Vector2D end, double t) {
        //Define t constants
        double u = 1 - t;
        double tt = t * t;
        double ttt = t * t * t;
        double uu = u * u;
        double uuu = u * u * u;

        //calculate X and Y coordinates by incrementing t between the points
        double x = uuu * start.getX() + 3 * uu * t * controlFirst.getX() + 3 * u * tt * controlSecond.getX() + ttt * end.getX();
        double y = uuu * start.getY() + 3 * uu * t * controlFirst.getY() + 3 * u * tt * controlSecond.getY() + ttt * end.getY();

        //return X and Y in a custom position class
        return new Vector2D(x, y);
    }

    /**Line gen equations method*/
    private Vector2D calculateLine(Vector2D start, Vector2D end, double t) {
        //Define t constants
        double u = 1 - t;

        //calculate X and Y coordinates by incrementing t between the points
        double x = start.getX() * u + end.getX() * t;
        double y = start.getY() * u + end.getY() * t;

        //return X and Y in a custom position class
        return new Vector2D(x, y);
    }

    private Vector2D calculateFirstDerivative(Vector2D start, Vector2D control, Vector2D end, double t) {
        double u = 1 - t;

        double x = 2 * u * (control.getX() - start.getX()) + 2 * t * (control.getX() - end.getX());
        double y = 2 * u * (control.getY() - start.getY()) + 2 * t * (control.getY() - end.getY());

        return new Vector2D(x, y);
    }

    // Method to calculate the second derivative of the quadratic Bezier curve
    private Vector2D calculateSecondDerivative(Vector2D start, Vector2D control, Vector2D end, double t) {
        double x = 2 * (end.getX() - 2 * control.getX() + start.getX());
        double y = 2 * (end.getY() - 2 * control.getY() + start.getY());

        return new Vector2D(x, y);
    }

    public double calculateRadius(Vector2D start, Vector2D control, Vector2D end, double t) {

        Vector2D firstDerivative = calculateFirstDerivative(start, control, end, t);
        Vector2D secondDerivative = calculateSecondDerivative(start, control, end, t);

        double numerator = Math.pow(firstDerivative.getNorm(), 3);
        double denominator = Math.pow(secondDerivative.getNorm(), 2);

        // Check for potential division by zero
        if (denominator != 0) {
            return numerator / denominator;
        } else {
            // Handle the case where denominator is zero (or close to zero)
            // This could happen if the second derivative is very small
            // You may want to handle this case based on your specific requirements
            return Double.POSITIVE_INFINITY; // or another suitable value
        }

    }

}
