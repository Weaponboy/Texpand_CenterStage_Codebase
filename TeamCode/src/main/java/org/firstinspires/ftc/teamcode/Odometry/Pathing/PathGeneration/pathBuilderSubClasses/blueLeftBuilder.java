package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Blue_Points_Overlap;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class blueLeftBuilder extends pathBuilderMain{

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * DP = drop purple pixel, DY = drop yellow pixel, C = collect pixels, D = deliver pixels from stack
     * S = start point, C = control point, CT = control point two, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * F = first prop pos, S = second prop pos, T = third prop pos
     * */

    /**if it is calling the getRealCoords method it has the correct values*/

    /**
     * drop purple pixel
     * */

    //first pos
    Vector2D DPS1F = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1F = new Vector2D(getRealCoords(241), getRealCoords(45));
    Vector2D DPCT1F = new Vector2D(getRealCoords(188), getRealCoords(83));
    Vector2D DPE1F = new Vector2D(getRealCoords(305), getRealCoords(82.5));

    //second pos
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1S = new Vector2D(getRealCoords(220), getRealCoords(76));
    Vector2D DPCT1S = new Vector2D(getRealCoords(170), getRealCoords(65));
    Vector2D DPE1S = new Vector2D(getRealCoords(300), getRealCoords(99));

    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1T = new Vector2D(getRealCoords(223), getRealCoords(79));
    Vector2D DPCT1T = new Vector2D(getRealCoords(164), getRealCoords(110));
    Vector2D DPE1T = new Vector2D(getRealCoords(305), getRealCoords(105));

    /**first position*/
    Vector2D DS1F = new Vector2D(getRealCoords(300), getRealCoords(121));

    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(263), getRealCoords(162));
    Vector2D CE1F = new Vector2D(getRealCoords(179), getRealCoords(152));

    //second segment
    Vector2D CS2F = CE1F;
    Vector2D CC2F = new Vector2D(getRealCoords(50), getRealCoords(150));


    public enum Position {
        left,
        center,
        right
    }

    public enum pathSplit {
        first,
        second
    }

    public enum Section {
        preload,
        collect,
        deliver
    }

    public void buildPathLine(Vector2D startPos, Vector2D targetPos){

        buildLineSegment(startPos, targetPos);

        pathBuilder(originalPath);

        motionProfile();

    }

    public void buildPath(Section section){

        switch (section) {
            case collect:
                Collect();
                break;
            case deliver:
                Deliver();
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    public void buildPath(Position propPosition, Section section){

        switch (section) {
            case preload:
                switch (propPosition) {
                    case left:
                        firstPositionPreload();
                        break;
                    case right:
                        thirdPositionPreload();
                        break;
                    case center:
                        secondPositionPreload();
                        break;
                    default:
                }
                break;
            case collect:
                Collect();
                break;
            case deliver:
                Deliver();
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    public void buildPath(Position propPosition, Section section, redRightBuilder.pathSplit pathsplit){

        switch (section) {
            case preload:
                switch (propPosition) {
                    case left:
                        firstPositionPreload();
                        break;
                    case right:
                        thirdPositionPreload();
                        break;
                    case center:
                        secondPositionPreload();
                        break;
                    default:
                }
                break;
            case collect:
                Collect();
                break;
            case deliver:
                Deliver();
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    /**First Position*/
    private void firstPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1F, DPC1F, DPCT1F,  DPE1F);

    }

    /**First Position*/
    private void Collect(){

        buildCurveSegment(CS1F, CC1F, CE1F);

        buildPathLine(CS2F, CC2F);

    }

    private void Deliver(){

        buildLineSegment(CC2F, CS2F);

        buildCurveSegment(CE1F, CC1F, DS1F);

    }

    /**second Position*/
    private void secondPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1S, DPC1S, DPCT1S, DPE1S);

    }

    /**third Position*/
    private void thirdPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1T, DPC1T, DPCT1T, DPE1T);

    }


}
