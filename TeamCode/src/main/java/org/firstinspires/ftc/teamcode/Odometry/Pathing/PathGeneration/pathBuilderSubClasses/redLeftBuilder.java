package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Red_Points_Overlap;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class redLeftBuilder extends pathBuilderMain{

    Vector2D startPos = new Vector2D(getRealCoords(90), getRealCoords(337));

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

    Vector2D DPS1T = startPos;
    Vector2D DPC1T = new Vector2D(getRealCoords(105), getRealCoords(245));
    Vector2D DPE1T = new Vector2D(getRealCoords(66), getRealCoords(262));

    //second pos
    Vector2D DPS1S = startPos;
    Vector2D DPC1S = new Vector2D(getRealCoords(100), getRealCoords(265));
    Vector2D DPE1S = new Vector2D(getRealCoords(66), getRealCoords(288));

    //third pos
    Vector2D DPS1F = startPos;
    Vector2D DPE1F = new Vector2D(getRealCoords(74), getRealCoords(286));

    /**drop yellow pixel*/

    /**drop yellow first*/
    Vector2D DYS1F = DPE1T;
    Vector2D DYC1F = new Vector2D(getRealCoords(15), getRealCoords(293));
    Vector2D DYE1F = new Vector2D(getRealCoords(68), getRealCoords(206));

    Vector2D DYS2F = DYE1F;
    Vector2D DYC2F = new Vector2D(getRealCoords(132), getRealCoords(174));
    Vector2D DYE2F = new Vector2D(getRealCoords(202), getRealCoords(195));

    Vector2D DYS3F = DYE2F;
    Vector2D DYC3F = new Vector2D(getRealCoords(285), getRealCoords(219));
    Vector2D DYE3F = new Vector2D(getRealCoords(305), getRealCoords(280));

    /**drop yellow second*/
    Vector2D DYS1S = DPE1S;
    Vector2D DYC1S = new Vector2D(getRealCoords(31), getRealCoords(324));
    Vector2D DYE1S = new Vector2D(getRealCoords(45), getRealCoords(240));

    Vector2D DYS2S = DYE1S;
    Vector2D DYC2S = new Vector2D(getRealCoords(67), getRealCoords(171));
    Vector2D DYE2S = new Vector2D(getRealCoords(236), getRealCoords(211));

    Vector2D DYS3S = DYE2S;
    Vector2D DYC3S = new Vector2D(getRealCoords(293), getRealCoords(224));
    Vector2D DYE3S = new Vector2D(getRealCoords(305), getRealCoords(265));

    /**drop yellow third*/

    Vector2D DYS1T = DPE1F;
    Vector2D DYC1T = new Vector2D(getRealCoords(95), getRealCoords(308));
    Vector2D DYE1T = new Vector2D(getRealCoords(85), getRealCoords(218));

    Vector2D DYS2T = DYE1T;
    Vector2D DYC2T = new Vector2D(getRealCoords(76), getRealCoords(162));
    Vector2D DYE2T = new Vector2D(getRealCoords(203), getRealCoords(187));

    Vector2D DYS3T = DYE2T;
    Vector2D DYC3T = new Vector2D(getRealCoords(276), getRealCoords(202));
    Vector2D DYE3T = new Vector2D(getRealCoords(305), getRealCoords(250));

    /**first position*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(270));
    Vector2D CC1F = new Vector2D(getRealCoords(284), getRealCoords(213));
    Vector2D CE1F = new Vector2D(getRealCoords(179), getRealCoords(208));

    //second segment
    Vector2D CS2F = CE1F;
    Vector2D CC2F = new Vector2D(getRealCoords(107), getRealCoords(211));
    Vector2D CE2F = new Vector2D(getRealCoords(84), getRealCoords(200));

    Vector2D CS3F = CE2F;
    Vector2D CC3F = new Vector2D(getRealCoords(37), getRealCoords(179));
    Vector2D CE3F = new Vector2D(getRealCoords(47), getRealCoords(240));


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

    public enum pixelColor {
        purple,
        yellow,
    }

    public void buildPath(redLeftBuilder.Position propPosition, redLeftBuilder.Section section){

        switch (section) {
            case preload:
                switch (propPosition) {
                    case left:
                        firstPositionPreloadPurple();
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

    public void buildPath(redLeftBuilder.Position propPosition, redLeftBuilder.Section section, redLeftBuilder.pixelColor color){

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

    public void buildPath(redLeftBuilder.Section section){

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

    /**
     * First Position
     * */

    private void firstPositionPreloadPurple(){

        // drop purple pixel
        buildLineSegment(DPS1F, DPE1F);

    }

    private void firstPositionPreloadYellow(){

        // drop yellow pixel
        buildCurveSegment(DYS1T, DYC1T, DYE1T);

        buildCurveSegment(DYS2T, DYC2T, DYE2T);

        buildCurveSegment(DYS3T, DYC3T, DYE3T);

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
        buildCurveSegment(DPS1S, DPC1S, DPE1S);

        // drop yellow pixel
        buildCurveSegment(DYS1S, DYC1S, DYE1S);

        buildCurveSegment(DYS2S, DYC2S, DYE2S);

        buildCurveSegment(DYS3S, DYC3S, DYE3S);

    }

    /**
     * third Position
     * */

    private void thirdPositionPreload(){

        buildCurveSegment(DPS1T, DPC1T, DPE1T);

        buildCurveSegment(DYS1F, DYC1F, DYE1F);

        buildCurveSegment(DYS2F, DYC2F, DYE2F);

        buildCurveSegment(DYS3F, DYC3F, DYE3F);

    }

}
