package org.firstinspires.ftc.teamcode.hardware.Odometry.Pathing.PathingPower;

public class PathingPower {

    private double vertical;
    private double horizontal;

    public PathingPower(double vertical, double horizontal){
        this.vertical = vertical;
        this.horizontal = horizontal;
    }

    public PathingPower() {
        this(0, 0);
    }

    public PathingPower set(double vertical, double horizontal) {
        this.vertical = vertical;
        this.horizontal = horizontal;
        return this;
    }

    public double getVertical(){
        return vertical;
    }

    public double getHorizontal(){
        return horizontal;
    }

}
