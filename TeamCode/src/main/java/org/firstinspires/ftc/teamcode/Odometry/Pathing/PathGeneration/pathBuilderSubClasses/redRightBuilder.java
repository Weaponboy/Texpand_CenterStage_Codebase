package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Red_Points_Overlap;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class redRightBuilder extends pathBuilderMain implements Red_Points_Overlap {

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * DP = drop purple pixel, DY = drop yellow pixel, C = collect pixels, D = deliver pixels from stack
     * S = start point, C = control point, CT = control point two, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * F = first prop pos, S = second prop pos, T = third prop pos
     * */

    /**if it is calling the getRealCoords method it has the correct values*/

    /**drop purple pixel*/

    Vector2D DPS1F = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPE1F = new Vector2D(getRealCoords(210), getRealCoords(270));

    //second pos
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1S = new Vector2D(getRealCoords(204), getRealCoords(245));
    Vector2D DPE1S = new Vector2D(getRealCoords(230), getRealCoords(301));

    //third pos
    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1T = new Vector2D(getRealCoords(210), getRealCoords(265));
    Vector2D DPE1T = new Vector2D(getRealCoords(227), getRealCoords(303));

    /**drop yellow pixel*/

    //drop yellow pixel first
    Vector2D DYS1F = new Vector2D(DPE1F.getX(), DPE1F.getY());
    Vector2D DYE1F = new Vector2D(getRealCoords(296), getRealCoords(260));

    //drop yellow pixel second
    Vector2D DYS1S = new Vector2D(DPE1S.getX(), DPE1S.getY());
    Vector2D DYC1S = new Vector2D(getRealCoords(210), getRealCoords(325));
    Vector2D DYE1S = new Vector2D(getRealCoords(296), getRealCoords(275));

    //drop yellow pixel third
    Vector2D DYS1T = new Vector2D(DPE1T.getX(), DPE1T.getY());
    Vector2D DYC1T = new Vector2D(getRealCoords(233), getRealCoords(341));
    Vector2D DYE1T = new Vector2D(getRealCoords(296), getRealCoords(290));


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

    public void buildPath(Position propPosition, Section section){

        switch (section) {
            case preload:
                switch (propPosition) {
                    case left:
                        firstPositionPreload1();
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
                switch (propPosition) {
                    case left:
                        firstPositionCollect();
                        break;
                    case right:
                        thirdPositionCollect();
                        break;
                    case center:
                        secondPositionCollect();
                        break;
                    default:
                }
                break;
            case deliver:
                switch (propPosition) {
                    case left:
                        firstPositionDeliver();
                        break;
                    case right:
                        thirdPositionDeliver();
                        break;
                    case center:
                        secondPositionDeliver();
                        break;
                    default:
                }
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    public void buildPath(Position propPosition, Section section, pathSplit pathsplit){

        switch (section) {
            case preload:
                switch (propPosition) {
                    case left:
                        switch (pathsplit){
                            case first:
                                firstPositionPreload1();
                                break;
                            case second:
                                firstPositionPreload2();
                                break;
                            default:
                        }
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
                switch (propPosition) {
                    case left:
                        firstPositionCollect();
                        break;
                    case right:
                        thirdPositionCollect();
                        break;
                    case center:
                        secondPositionCollect();
                        break;
                    default:
                }
                break;
            case deliver:
                switch (propPosition) {
                    case left:
                        firstPositionDeliver();
                        break;
                    case right:
                        thirdPositionDeliver();
                        break;
                    case center:
                        secondPositionDeliver();
                        break;
                    default:
                }
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }


    /**
     * First Position
     * */

    private void firstPositionPreload1(){
        buildLineSegment(DPS1F, DPE1F);
    }

    private void firstPositionPreload2(){
//        //drop yellow pixel
        buildLineSegment(DYS1F, DYE1F);
    }

    private void firstPositionCollect(){

        buildCurveSegment(CS1F, CC1F, CE1F);

        buildCurveSegment(CS2F, CC2F, CE2F);

    }

    private void firstPositionDeliver(){

        buildCurveSegment(DS1F, DC1F, DE1F);

        buildCurveSegment(DS2F, DC2F, DE2F);
    }

    /**
     * Second Position
     * */

    private void secondPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1S, DPC1S, DPE1S);

        // drop yellow pixel
        buildCurveSegment(DYS1S, DYC1S, DYE1S);

    }

    private void secondPositionCollect(){

        buildCurveSegment(CS1S, CC1S, CE1S);

        buildCurveSegment(CS2S, CC2S, CE2S);

    }

    private void secondPositionDeliver(){

        buildCurveSegment(DS1S, DC1S, DE1S);

        buildCurveSegment(DS2S, DC2S, DE2S);

    }

    /**
     * Third Position
     * */

    private void thirdPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1T, DPC1T, DPE1T);

        // drop yellow pixel
        buildCurveSegment(DYS1T, DYC1T, DYE1T);

    }

    private void thirdPositionCollect(){

        buildCurveSegment(CS1T, CC1T, CE1T);

        buildCurveSegment(CS2T, CC2T, CE2T);


    }

    private void thirdPositionDeliver(){

        buildCurveSegment(DS1T, DC1T, DE1T);

        buildCurveSegment(DS2T, DC2T, DE2T);

    }


}
