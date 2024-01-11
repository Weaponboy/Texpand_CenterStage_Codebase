package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class blueLeftBuilder extends pathBuilderMain {

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
    Vector2D DPC1F = new Vector2D(getRealCoords(210), getRealCoords(95));
    Vector2D DPE1F = new Vector2D(getRealCoords(227), getRealCoords(57));

    //second pos
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1S = new Vector2D(getRealCoords(204), getRealCoords(115));
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
    Vector2D DYE1T = new Vector2D(getRealCoords(300), getRealCoords(105));

    /**
     * collect white pixels from stack
     * */

    /**first position*/
    Vector2D CS1F = new Vector2D(DYE1F.getX(), DYE1F.getY());
    Vector2D CC1F = new Vector2D(getRealCoords(292), getRealCoords(166));
    Vector2D CE1F = new Vector2D(getRealCoords(180), getRealCoords(150));

    //second segment
    Vector2D CS2F = new Vector2D(getRealCoords(180), getRealCoords(150));
    Vector2D CE2F = new Vector2D(getRealCoords(33), getRealCoords(150));

    /**second position*/
    Vector2D CS1S = new Vector2D(DYE1S.getX(), DYE1S.getY());
    Vector2D CC1S = new Vector2D(getRealCoords(292), getRealCoords(166));
    Vector2D CE1S = new Vector2D(getRealCoords(180), getRealCoords(150));

    //second segment
    Vector2D CS2S = new Vector2D(getRealCoords(180), getRealCoords(150));
    Vector2D CE2S = new Vector2D(getRealCoords(33), getRealCoords(150));

    /**third position*/
    Vector2D CS1T = new Vector2D(DYE1T.getX(), DYE1T.getY());
    Vector2D CC1T = new Vector2D(getRealCoords(294), getRealCoords(149));
    Vector2D CE1T = new Vector2D(getRealCoords(181), getRealCoords(193));

    //second segment
    Vector2D CS2T = new Vector2D(CE1T.getX(), CE1T.getY());
    Vector2D CC2T = new Vector2D(getRealCoords(30), getRealCoords(249));
    Vector2D CE2T = new Vector2D(getRealCoords(41), getRealCoords(135));

    /**
     * deliver points
     * */

    /**first position*/
    Vector2D DS1F = new Vector2D(CE2F.getX(), CE2F.getY());
    Vector2D DC1F = new Vector2D(getRealCoords(84), getRealCoords(189));
    Vector2D DE1F = new Vector2D(getRealCoords(179), getRealCoords(176));

    Vector2D DS2F = new Vector2D(DE1F.getX(), DE1F.getY());
    Vector2D DC2F = new Vector2D(getRealCoords(293), getRealCoords(160));
    Vector2D DE2F = new Vector2D(getRealCoords(296), getRealCoords(90));

    /**second position*/
    Vector2D DS1S = new Vector2D(CE2S.getX(), CE2S.getY());
    Vector2D DC1S = new Vector2D(getRealCoords(84), getRealCoords(189));
    Vector2D DE1S = new Vector2D(getRealCoords(179), getRealCoords(176));

    Vector2D DS2S = new Vector2D(DE1S.getX(), DE1S.getY());
    Vector2D DC2S = new Vector2D(getRealCoords(293), getRealCoords(160));
    Vector2D DE2S = new Vector2D(getRealCoords(296), getRealCoords(90));

    /**third position*/
    Vector2D DS1T = new Vector2D(CE2T.getX(), CE2T.getY());
    Vector2D DC1T = new Vector2D(getRealCoords(84), getRealCoords(189));
    Vector2D DE1T = new Vector2D(getRealCoords(179), getRealCoords(176));

    Vector2D DS2T = new Vector2D(DE1T.getX(), DE1T.getY());
    Vector2D DC2T = new Vector2D(getRealCoords(293), getRealCoords(160));
    Vector2D DE2T = new Vector2D(getRealCoords(296), getRealCoords(90));

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
                        firstPositionPreload();
                        break;
                    case right:
                        thirdPositionDeliver();
                        break;
                    case center:
                        secondPositionPreload();
                        break;
                    default:
                }
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
                        firstPositionPreload();
                        break;
                    case right:
                        thirdPositionDeliver();
                        break;
                    case center:
                        secondPositionPreload();
                        break;
                    default:
                }
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
    private void firstPositionCollect(){

        buildCurveSegment(CS1F, CC1F, CE1F);

        buildLineSegment(CS2F, CE2F);

    }

    /**second Position*/
    private void secondPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1S, DPC1S, DPE1S);

        // drop yellow pixel
        buildCurveSegment(DYS1S, DYC1S, DYE1S);

    }

    /**second Position*/
    private void secondPositionCollect(){

        buildCurveSegment(CS1S, CC1S, CE1S);

        buildLineSegment(CS2S, CE2S);

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

    /**third Position*/
    private void thirdPositionCollect(){

        buildCurveSegment(CS1T, CC1T, CE1T);

        buildCurveSegment(CS2T, CC2T, CE2T);

    }

    /**third Position*/
    private void thirdPositionDeliver(){

        buildCurveSegment(DS1T, DC1T, DE1T);

        buildCurveSegment(DS2T, DC2T, DE2T);

    }

}
