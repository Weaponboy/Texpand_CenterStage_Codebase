package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class redRightBuilder extends pathBuilderMain {

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * DP = drop purple pixel, DY = drop yellow pixel, C = collect pixels, D = deliver pixels from stack
     * S = start point, C = control point, CT = control point two, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * F = first prop pos, S = second prop pos, T = third prop pos
     * */

    /**if it is calling the getRealCoords method it has the correct values*/

    /**drop purple pixel*/

    //first pos DONE!!!!!
    Vector2D DPS1F = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1F = new Vector2D(getRealCoords(252), getRealCoords(274));
    Vector2D DPC21F = new Vector2D(getRealCoords(136), getRealCoords(269));
    Vector2D DPE1F = new Vector2D(getRealCoords(250), getRealCoords(258));

    //second pos
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1S = new Vector2D(getRealCoords(210), getRealCoords(222));
    Vector2D DPE1S = new Vector2D(getRealCoords(210), getRealCoords(297));

    //third pos
    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1T = new Vector2D(getRealCoords(210), getRealCoords(208));
    Vector2D DPE1T = new Vector2D(getRealCoords(229), getRealCoords(288));

    /**drop yellow pixel*/

    //drop yellow pixel first
    Vector2D DYS1F = new Vector2D(DPE1F.getX(), DPE1F.getY());
    Vector2D DYE1F = new Vector2D(getRealCoords(300), getRealCoords(255));

    //drop yellow pixel second
    Vector2D DYS1S = new Vector2D(DPE1S.getX(), DPE1S.getY());
    Vector2D DYC1S = new Vector2D(getRealCoords(210), getRealCoords(325));
    Vector2D DYE1S = new Vector2D(getRealCoords(300), getRealCoords(270));

    //drop yellow pixel third
    Vector2D DYS1T = new Vector2D(DPE1T.getX(), DPE1T.getY());
    Vector2D DYC1T = new Vector2D(getRealCoords(233), getRealCoords(325));
    Vector2D DYE1T = new Vector2D(getRealCoords(300), getRealCoords(286));

    /**collect white pixels from stack, These are also for delivering the white pixels but just reversed*/

    /*first position*/
    Vector2D CS1F = new Vector2D(DYE1F.getX(), DYE1F.getY());
    Vector2D CC1F = new Vector2D(getRealCoords(292), getRealCoords(194));
    Vector2D CE1F = new Vector2D(getRealCoords(180), getRealCoords(210));

    //first segment
    Vector2D CS2F = new Vector2D(getRealCoords(180), getRealCoords(210));
    Vector2D CE2F = new Vector2D(getRealCoords(33), getRealCoords(210));

    /*second position*/
    Vector2D CS1S = new Vector2D(DYE1S.getX(), DYE1S.getY());
    Vector2D CC1S = new Vector2D(getRealCoords(292), getRealCoords(194));
    Vector2D CE1S = new Vector2D(getRealCoords(180), getRealCoords(210));

    //second segment
    Vector2D CS2S = new Vector2D(getRealCoords(180), getRealCoords(210));
    Vector2D CE2S = new Vector2D(getRealCoords(33), getRealCoords(210));

    /*third position*/
    Vector2D CS1T = new Vector2D(DYE1T.getX(), DYE1T.getY());
    Vector2D CC1T = new Vector2D(getRealCoords(292), getRealCoords(194));
    Vector2D CE1T = new Vector2D(getRealCoords(180), getRealCoords(210));

    //third segment
    Vector2D CS2T = new Vector2D(getRealCoords(180), getRealCoords(210));
    Vector2D CE2T = new Vector2D(getRealCoords(33), getRealCoords(210));

    public enum Position {
        left,
        center,
        right
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
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    /**First Position*/
    private void firstPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1F, DPC1F, DPC21F, DPE1F);

        // drop yellow pixel
        buildLineSegment(DYS1F, DYE1F);

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
    private void thirdPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1T, DPC1T, DPE1T);

        // drop yellow pixel
        buildCurveSegment(DYS1T, DYC1T, DYE1T);

        buildCurveSegment(CS1T, CC1T, CE1T);

        buildLineSegment(CS2T, CE2T);

    }

    /**third Position*/
    private void thirdPositionCollect(){

        buildCurveSegment(CS1T, CC1T, CE1T);

        buildLineSegment(CS2T, CE2T);

    }

}
