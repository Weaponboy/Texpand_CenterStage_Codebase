package org.firstinspires.ftc.teamcode.Constants_and_Setpoints;

public class PathingPoint {

    public double x;
    public double y;
    public double heading;

    public PathingPoint(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getHeading(){
        return heading;
    }

}
