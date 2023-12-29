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

            //increment t slightly, i prefer having this very low just in increase accuracy
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

}
