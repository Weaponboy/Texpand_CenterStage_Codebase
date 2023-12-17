package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility;

public class RobotPower {

    private double vertical;
    private double horizontal;
    private double pivot;

    public RobotPower(double vertical, double horizontal, double pivot){
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
    }

    public RobotPower() {
        this(0, 0, 0);
    }

    public RobotPower set(double vertical, double horizontal, double pivot) {
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        return this;
    }

    public double getVertical(){
        return vertical;
    }

    public double getHorizontal(){
        return horizontal;
    }

    public double getPivot(){
        return pivot;
    }

}
