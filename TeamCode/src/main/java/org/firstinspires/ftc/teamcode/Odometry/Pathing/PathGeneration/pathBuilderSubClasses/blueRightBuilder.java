package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class blueRightBuilder extends pathBuilderMain {

    Vector2D startPos = new Vector2D(getRealCoords(90), getRealCoords(23));

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * DP = drop purple pixel, DY = drop yellow pixel, C = collect pixels, D = deliver pixels from stack
     * S = start point, C = control point, CT = control point two, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * F = first prop pos, S = second prop pos, T = third prop pos
     * */

    /**if it is calling the getRealCoords method it has the correct values*/

    /**FIRST POSITION ALL POINTS*/
    /**drop purple pixel*/

    Vector2D DPS1F = startPos;
    Vector2D DPE1F = new Vector2D(getRealCoords(90), getRealCoords(90));

    //second pos
    Vector2D DPS1S = startPos;
    Vector2D DPC1S = new Vector2D(getRealCoords(84), getRealCoords(115));
    Vector2D DPE1S = new Vector2D(getRealCoords(70), getRealCoords(60));

    //third pos
    Vector2D DPS1T = startPos;
    Vector2D DPE1T = new Vector2D(getRealCoords(74), getRealCoords(74));

    /**drop yellow pixel*/

    /**drop yellow first*/
    Vector2D DYS1F = DPE1F;
    Vector2D DYC1F = new Vector2D(getRealCoords(30), getRealCoords(140));
    Vector2D DYE1F = new Vector2D(getRealCoords(103), getRealCoords(174));

    Vector2D DYS2F = DYE1F;
    Vector2D DYC2F = new Vector2D(getRealCoords(158), getRealCoords(188));
    Vector2D DYE2F = new Vector2D(getRealCoords(240), getRealCoords(157));

    Vector2D DYS3F = DYE2F;
    Vector2D DYC3F = new Vector2D(getRealCoords(293), getRealCoords(136));
    Vector2D DYE3F = new Vector2D(getRealCoords(300), getRealCoords(75));

    /**drop yellow second*/
    Vector2D DYS1S = DPE1S;
    Vector2D DYC1S = new Vector2D(getRealCoords(35), getRealCoords(0));
    Vector2D DYE1S = new Vector2D(getRealCoords(35), getRealCoords(90));

    Vector2D DYS2S = DYE1S;
    Vector2D DYC2S = new Vector2D(getRealCoords(24), getRealCoords(205));
    Vector2D DYE2S = new Vector2D(getRealCoords(240), getRealCoords(157));

    Vector2D DYS3S = DYE2S;
    Vector2D DYC3S = new Vector2D(getRealCoords(293), getRealCoords(136));
    Vector2D DYE3S = new Vector2D(getRealCoords(300), getRealCoords(90));

    /**drop yellow third*/

    Vector2D DYS1T = DPE1T;
    Vector2D DYC1T = new Vector2D(getRealCoords(111), getRealCoords(68));
    Vector2D DYE1T = new Vector2D(getRealCoords(86), getRealCoords(142));

    Vector2D DYS2T = DYE1T;
    Vector2D DYC2T = new Vector2D(getRealCoords(64), getRealCoords(194));
    Vector2D DYE2T = new Vector2D(getRealCoords(180), getRealCoords(181));

    Vector2D DYS3T = DYE2T;
    Vector2D DYC3T = new Vector2D(getRealCoords(270), getRealCoords(167));
    Vector2D DYE3T = new Vector2D(getRealCoords(300), getRealCoords(105));

    /**collect white pixels from stack, These are also for delivering the white pixels but just reversed*/

    /*first position*/
    Vector2D CS1F = DYE1F;
    Vector2D CC1F = new Vector2D(getRealCoords(292), getRealCoords(194));
    Vector2D CE1F = new Vector2D(getRealCoords(180), getRealCoords(210));

    //first segment
    Vector2D CS2F = new Vector2D(getRealCoords(180), getRealCoords(210));
    Vector2D CE2F = new Vector2D(getRealCoords(41), getRealCoords(210));

    /*second position*/
    Vector2D CS1S = new Vector2D(DYE1S.getX(), DYE1S.getY());
    Vector2D CC1S = new Vector2D(getRealCoords(292), getRealCoords(194));
    Vector2D CE1S = new Vector2D(getRealCoords(180), getRealCoords(210));

    //second segment
    Vector2D CS2S = new Vector2D(getRealCoords(180), getRealCoords(210));
    Vector2D CE2S = new Vector2D(getRealCoords(41), getRealCoords(210));

    /*third position*/
    Vector2D CS1T = new Vector2D(DYE1T.getX(), DYE1T.getY());
    Vector2D CC1T = new Vector2D(getRealCoords(292), getRealCoords(194));
    Vector2D CE1T = new Vector2D(getRealCoords(180), getRealCoords(210));

    //third segment
    Vector2D CS2T = new Vector2D(getRealCoords(180), getRealCoords(210));
    Vector2D CE2T = new Vector2D(getRealCoords(38), getRealCoords(210));


    public enum Position {
        left,
        center,
        right
    }

    public enum pixelColor {
        purple,
        yellow,
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
                        firstPositionPreloadPurple();
                        break;
                    case right:
                        thirdPositionPreloadPurple();
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
                        secondPositionDeliver();
                        break;
                    case center:
                        thirdPositionDeliver();
                        break;
                    default:
                }
                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    public void buildPath(Position propPosition, Section section, pixelColor color){

        switch (section) {
            case preload:
                switch (propPosition) {
                    case left:
                        switch (color){
                            case purple:
                                firstPositionPreloadPurple();
                                break;
                            case yellow:
                                firstPositionPreloadYellow();
                                break;
                            default:
                        }
                        break;
                    case right:
                        switch (color){
                            case purple:
                                thirdPositionPreloadPurple();
                                break;
                            case yellow:
                                thirdPositionPreloadYellow();
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

    private void firstPositionPreloadPurple(){

        // drop purple pixel
        buildLineSegment(DPS1F, DPE1F);

    }

    private void firstPositionPreloadYellow(){

        // drop yellow pixel
        buildCurveSegment(DYS1F, DYC1F, DYE1F);

        buildCurveSegment(DYS2F, DYC2F, DYE2F);

        buildCurveSegment(DYS3F, DYC3F, DYE3F);

    }

    /**First Position*/
    private void firstPositionCollect(){

        buildCurveSegment(CS1F, CC1F, CE1F);

        buildLineSegment(CS2F, CE2F);

    }

    /**First Position*/
    private void firstPositionDeliver(){

        buildCurveSegment(CS1F, CC1F, CE1F);

        buildLineSegment(CS2F, CE2F);

    }

    /**
     * Second Position
     * */

    private void secondPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1S, DPC1S, DPE1S);

        // drop yellow pixel
        buildCurveSegment(DYS1S, DYC1S, DYE1S);

        buildCurveSegment(DYS2S, DYC2S, DYE2S);

        buildCurveSegment(DYS3S, DYC3S, DYE3S);

    }

    /**second Position*/
    private void secondPositionCollect(){

        buildCurveSegment(CS1S, CC1S, CE1S);

        buildLineSegment(CS2S, CE2S);

    }

    private void secondPositionDeliver(){

        buildCurveSegment(CS1S, CC1S, CE1S);

        buildLineSegment(CS2S, CE2S);

    }


    /**
     * third Position
     * */
    private void thirdPositionPreloadPurple(){

        // drop purple pixel
        buildLineSegment(DPS1T, DPE1T);

    }

    private void thirdPositionPreloadYellow(){

        buildCurveSegment(DYS1T, DYC1T, DYE1T);

        buildCurveSegment(DYS2T, DYC2T, DYE2T);

        buildCurveSegment(DYS3T, DYC3T, DYE3T);

    }

    /**third Position*/
    private void thirdPositionCollect(){

        buildCurveSegment(CS1T, CC1T, CE1T);

        buildLineSegment(CS2T, CE2T);

    }

    private void thirdPositionDeliver(){

        buildCurveSegment(CS1T, CC1T, CE1T);

        buildLineSegment(CS2T, CE2T);

    }

}
