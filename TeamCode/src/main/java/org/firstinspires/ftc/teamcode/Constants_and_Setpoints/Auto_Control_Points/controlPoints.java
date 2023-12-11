package org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Auto_Control_Points;

import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.Vector2D;

public class controlPoints {

    /**
     * Drop purple position control points
     * */

    /**blue right start*/
    public Vector2D dropPurpleBlueRightStartPosition = new Vector2D(90, 23);
    public Vector2D dropPurpleBlueRightEndPosition = new Vector2D(90, 90);

    /**blue right start*/
    public Vector2D dropPurpleBlueLeftStartPosition = new Vector2D(93, 23);
    public Vector2D dropPurpleBlueLeftEndPosition = new Vector2D(90, 90);

    /**red right start*/
    public Vector2D dropPurpleRedRightStartPosition = new Vector2D(210, 337);
    public Vector2D dropPurpleRedRightEndPosition = new Vector2D(210, 267);


    /**blue right start*/
    public Vector2D dropPurpleRedLeftStartPosition = new Vector2D(93, 23);
    public Vector2D dropPurpleRedLeftEndPosition = new Vector2D(90, 90);

    /**
     * Drop yellow position control points
     * */

    /**blue right*/
    public Vector2D sPSecondSeg = new Vector2D(90, 90);
    public Vector2D cPSecondSeg = new Vector2D(90, 160);
    public Vector2D ePSecondSeg = new Vector2D(154, 157);

    public Vector2D sPThirdSeg = new Vector2D(ePSecondSeg.getX(), ePSecondSeg.getY());
    public Vector2D cPThirdSeg = new Vector2D(302, 154);
    public static Vector2D ePThirdSeg = new Vector2D(305, 90);

    /**red right*/
    public Vector2D dropYellowRedRightStartPosition = new Vector2D(210, 267);
    public Vector2D dropYellowRedRightEndPosition = new Vector2D(298, 267);

    public Vector2D driveToCollectionRedRightStartPositionFirstSegment = new Vector2D(298, 267);
    public Vector2D driveToCollectionRedRightControlPositionFirstSegment = new Vector2D(290, 180);
    public Vector2D driveToCollectionRedRightEndPositionFirstSegment = new Vector2D(180, 210);

    public Vector2D driveToCollectionRedRightStartPositionSecondSegment = new Vector2D(180, 210);
    public Vector2D driveToCollectionRedRightEndPositionSecondSegment = new Vector2D(43, 210);

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
