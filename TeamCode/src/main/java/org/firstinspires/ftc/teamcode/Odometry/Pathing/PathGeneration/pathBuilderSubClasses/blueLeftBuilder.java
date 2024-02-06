package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Blue_Points_Overlap;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class blueLeftBuilder extends pathBuilderMain implements Blue_Points_Overlap {

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
    Vector2D DPC1F = new Vector2D(getRealCoords(205), getRealCoords(95));
    Vector2D DPE1F = new Vector2D(getRealCoords(227), getRealCoords(57));

    //second pos
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1S = new Vector2D(getRealCoords(204), getRealCoords(120));
    Vector2D DPE1S = new Vector2D(getRealCoords(230), getRealCoords(51));

    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPE1T = new Vector2D(getRealCoords(210), getRealCoords(90));

    /**
     * drop yellow pixel
     * */

    //drop yellow pixel first
    Vector2D DYS1F = new Vector2D(DPE1F.getX(), DPE1F.getY());
    Vector2D DYC1F = new Vector2D(getRealCoords(242), getRealCoords(26));
    Vector2D DYE1F = new Vector2D(getRealCoords(300), getRealCoords(75));

    //drop yellow pixel second
    Vector2D DYS1S = new Vector2D(DPE1S.getX(), DPE1S.getY());
    Vector2D DYC1S = new Vector2D(getRealCoords(210), getRealCoords(35));
    Vector2D DYE1S = new Vector2D(getRealCoords(300), getRealCoords(90));

    //drop yellow pixel first
    Vector2D DYS1T = new Vector2D(DPE1F.getX(), DPE1F.getY());
    Vector2D DYE1T = new Vector2D(getRealCoords(300), getRealCoords(110));

    public enum Position {
        left,
        center,
        right
    }

    public enum pathSplit {
        first,
        second,
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
                        thirdPositionPreload1();
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
                        switch (pathsplit){
                            case first:
                                thirdPositionPreload1();
                                break;
                            case second:
                                thirdPositionPreload2();
                                break;
                            default:
                        }
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
        buildCurveSegment(DPS1F, DPC1F,  DPE1F);

        // drop yellow pixel
        buildCurveSegment(DYS1F, DYC1F, DYE1F);

    }

    /**First Position*/
    private void Collect(){

        buildCurveSegment(CS1F, CC1F, CE1F);

        buildCurveSegment(CS2F, CC2F, CE2F);

    }

    private void Deliver(){

        buildCurveSegment(DS1F, DC1F, DE1F);

        buildCurveSegment(DS2F, DC2F, DE2F);

    }

    /**second Position*/
    private void secondPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1S, DPC1S, DPE1S);

        // drop yellow pixel
        buildCurveSegment(DYS1S, DYC1S, DYE1S);

    }

    /**third Position*/
    private void thirdPositionPreload1(){

        // drop purple pixel
        buildLineSegment(DPS1T, DPE1T);

    }

    /**third Position*/
    private void thirdPositionPreload2(){

        // drop yellow pixel
        buildLineSegment(DYS1T, DYE1T);

    }


}
