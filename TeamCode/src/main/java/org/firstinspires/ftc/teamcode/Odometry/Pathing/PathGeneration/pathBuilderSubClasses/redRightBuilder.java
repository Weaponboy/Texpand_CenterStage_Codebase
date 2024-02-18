package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Red_Points_Overlap;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class redRightBuilder extends pathBuilderMain{

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * DP = drop purple pixel, DY = drop yellow pixel, C = collect pixels, D = deliver pixels from stack
     * S = start point, C = control point, CT = control point two, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * F = first prop pos, S = second prop pos, T = third prop pos
     * */

    /**if it is calling the getRealCoords method it has the correct values*/

    /**drop purple pixel*/

    //first pos
    Vector2D DPS1F = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1F = new Vector2D(getRealCoords(233), getRealCoords(283));
    Vector2D DPCT1F = new Vector2D(getRealCoords(164), getRealCoords(250));
    Vector2D DPE1F = new Vector2D(getRealCoords(302), getRealCoords(290));

    //second pos
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1S = new Vector2D(getRealCoords(225), getRealCoords(284));
    Vector2D DPCT1S = new Vector2D(getRealCoords(180), getRealCoords(270));
    Vector2D DPE1S = new Vector2D(getRealCoords(305), getRealCoords(274));

    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1T = new Vector2D(getRealCoords(223), getRealCoords(281));
    Vector2D DPCT1T = new Vector2D(getRealCoords(164), getRealCoords(250));
    Vector2D DPE1T = new Vector2D(getRealCoords(305), getRealCoords(265));

    /**drop yellow pixel*/

    Vector2D DYS1F = new Vector2D(DPE1F.getX(), DPE1F.getY());
    Vector2D DYC1F = new Vector2D(getRealCoords(242), getRealCoords(334));
    Vector2D DYE1F = new Vector2D(getRealCoords(300), getRealCoords(285));

    //drop yellow pixel second
    Vector2D DYS1S = new Vector2D(DPE1S.getX(), DPE1S.getY());
    Vector2D DYC1S = new Vector2D(getRealCoords(210), getRealCoords(325));
    Vector2D DYE1S = new Vector2D(getRealCoords(300), getRealCoords(274));

    //drop yellow pixel third
    Vector2D DYS1T = new Vector2D(DPE1T.getX(), DPE1T.getY());
    Vector2D DYC1T = new Vector2D(getRealCoords(233), getRealCoords(341));
    Vector2D DYE1T = new Vector2D(getRealCoords(300), getRealCoords(289));

    /**first position*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(270));
    Vector2D CC1F = new Vector2D(getRealCoords(284), getRealCoords(213));
    Vector2D CE1F = new Vector2D(getRealCoords(179), getRealCoords(208));

    //second segment
    Vector2D CS2F = CE1F;
    Vector2D CC2F = new Vector2D(getRealCoords(107), getRealCoords(211));
    Vector2D CE2F = new Vector2D(getRealCoords(84), getRealCoords(200));

    Vector2D CS3F = CE2F;
    Vector2D CC3F = new Vector2D(getRealCoords(32), getRealCoords(179));
    Vector2D CE3F = new Vector2D(getRealCoords(42), getRealCoords(240));



    /**first position*/
    Vector2D DS1F = new Vector2D(CE2F.getX(), CE2F.getY());
    Vector2D DC1F = new Vector2D(getRealCoords(20), getRealCoords(209));
    Vector2D DE1F = new Vector2D(getRealCoords(96), getRealCoords(207));

    Vector2D DS2F = DE1F;
    Vector2D DC2F = new Vector2D(getRealCoords(290), getRealCoords(196));
    Vector2D DE2F = new Vector2D(getRealCoords(300), getRealCoords(270));



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

    public void buildPath(Position propPosition, Section section, pathSplit pathsplit){

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

    public void buildPathLine(Vector2D startPos, Vector2D targetPos){

        buildLineSegment(startPos, targetPos);

        pathBuilder(originalPath);

        motionProfile();
    }


    /**
     * First Position
     * */

    private void thirdPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1F, DPC1F,  DPE1F);

        // drop yellow pixel
        buildCurveSegment(DYS1F, DYC1F, DYE1F);

    }

    private void Collect(){

        buildCurveSegment(CS1F, CC1F, CE1F);

        buildCurveSegment(CS2F, CC2F, CE2F);

        buildCurveSegment(CS3F, CC3F, CE3F);

    }

    private void Deliver(){

        buildCurveSegment(CE3F, CC3F, CS3F);

        buildCurveSegment(CE2F, CC2F, CS2F);

        buildCurveSegment(CE1F, CC1F, CS1F);

    }

    /**
     * Second Position
     * */

    private void secondPositionPreload(){

        // drop purple pixel
        buildCurveSegment(DPS1S, DPC1S, DPCT1S, DPE1S);

    }

    /**
     * Third Position
     * */

    private void firstPositionPreload(){

        buildCurveSegment(DPS1T, DPC1T, DPCT1T, DPE1T);

    }


}
