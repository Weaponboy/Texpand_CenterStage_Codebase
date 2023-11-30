package org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Auto_Control_Points;

import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.Vector2D;

public class controlPoints {

    /**blue right start to backboard*/
    public Vector2D sPFirstSeg = new Vector2D(93, 23);
    public Vector2D ePFirstSeg = new Vector2D(90, 90);

    public Vector2D sPSecondSeg = new Vector2D(90, 90);
    public Vector2D cPSecondSeg = new Vector2D(90, 160);
    public Vector2D ePSecondSeg = new Vector2D(154, 157);

    public Vector2D sPThirdSeg = new Vector2D(ePSecondSeg.getX(), ePSecondSeg.getY());
    public Vector2D cPThirdSeg = new Vector2D(302, 154);
    public static Vector2D ePThirdSeg = new Vector2D(305, 90);

    /**test curve*/
    public Vector2D sPTest = new Vector2D(0, 0);
    public Vector2D cPTest = new Vector2D(75, 0);
    public Vector2D ePTest = new Vector2D(75, 75);

    public Vector2D sTest = new Vector2D(75, 75);
    public Vector2D eTest = new Vector2D(130, 50);

    public Vector2D intermediatePointToBackboard = new Vector2D(122, 183);
    public Vector2D intermediateControlToBackboard = new Vector2D(302, 183);
    public Vector2D dropAtBlueBackboard = new Vector2D(260, 75);
    public Vector2D dropAtRedBackboard = new Vector2D(260, 281);

    public Vector2D intermediatePointToCollection = new Vector2D(183, 183);
    public Vector2D intermediateControlToCollection = new Vector2D(248, 182);
    public Vector2D collectAtRed = new Vector2D(61, 61);
    public Vector2D collectAtBlue = new Vector2D(61, 304);

}
