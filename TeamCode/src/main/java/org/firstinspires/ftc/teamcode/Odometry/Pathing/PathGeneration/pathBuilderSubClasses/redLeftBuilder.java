package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class redLeftBuilder extends pathBuilderMain {

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * DP = drop purple pixel, DY = drop yellow pixel, C = collect pixels, D = deliver pixels from stack
     * S = start point, C = control point, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * */

    //drop purple pixel
    Vector2D DPS1 = new Vector2D(90, 337);
    Vector2D DPE1 = new Vector2D(90, 267);

    //drop yellow pixel
    //first segment
    public Vector2D DYS1 = new Vector2D(90, 90);
    public Vector2D DYC1 = new Vector2D(90, 160);
    public Vector2D DYE1 = new Vector2D(154, 157);

    //second segment
    public Vector2D DYS2 = new Vector2D(154, 157);
    public Vector2D DYC2 = new Vector2D(302, 154);
    public Vector2D DYE2 = new Vector2D(298, 90);

    //collect white pixels from stack, These are also for delivering the white pixels but just reversed

    //first segment
    Vector2D CS1 = new Vector2D(298, 267);
    Vector2D CC1 = new Vector2D(290, 180);
    Vector2D CE1 = new Vector2D(180, 210);

    //second segment
    Vector2D CS2 = new Vector2D(180, 210);
    Vector2D CE2 = new Vector2D(43, 210);

    public enum TargetPoint {
        dropPurple,
        dropYellow,
        collect ,
        dropWhite
    }

    public void buildPath(redRightBuilder.TargetPoint targetPoint){

        switch (targetPoint) {
            case dropPurple:
                dropPurple();
                break;
            case dropYellow:
                dropYellow();
                break;
            case collect:
                collectFromStack();
                break;
            case dropWhite:
                deliverWhitePixels();
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    public void buildPath(Vector2D SP, Vector2D CP, Vector2D EP){

        buildCurveSegment(SP, CP, EP);

        pathBuilder(originalPath);

        motionProfile();
    }

    /**Drop purple pixel*/
    private void dropPurple(){
        buildLineSegment(DPS1, DPE1);
    }

    /**Drop yellow pixel*/
    private void dropYellow(){
        buildLineSegment(DYS1, DYE1);
    }

    /**collect white pixel*/
    private void collectFromStack(){
        buildCurveSegment(CS1, CC1, CE1);
        buildLineSegment(CS2, CE2);
    }

    /**drop white pixel*/
    private void deliverWhitePixels(){
        //reversed use of collection points
        buildLineSegment(CE2, CS2);
        buildCurveSegment(CE1, CC1, CS1);
    }


}
