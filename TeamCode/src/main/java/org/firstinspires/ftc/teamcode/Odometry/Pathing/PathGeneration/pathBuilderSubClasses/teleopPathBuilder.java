package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class teleopPathBuilder extends pathBuilderMain {

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * BD = backdrop side segment, WC = wing and stack side segment, T = through truss segment
     * S = start point, C = control point, CT = control point two, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * F = first prop pos, S = second prop pos, T = third prop pos
     * */

    /**if it is calling the getRealCoords method it has the correct values*/

    /**drop purple pixel*/

    //through segments
    Vector2D T1Start = new Vector2D(120, 30);
    Vector2D T1End = new Vector2D(180, 30);

    Vector2D T2Start = new Vector2D(120, 90);
    Vector2D T2End = new Vector2D(180, 90);

    Vector2D T3Start = new Vector2D(120, 150);
    Vector2D T3End = new Vector2D(180, 150);

    Vector2D T4Start = new Vector2D(120, 210);
    Vector2D T4End = new Vector2D(180, 210);

    Vector2D T5Start = new Vector2D(120, 270);
    Vector2D T5End = new Vector2D(180, 270);

    Vector2D T6Start = new Vector2D(120, 330);
    Vector2D T6End = new Vector2D(180, 330);

    public enum Direction {
        toCollect,
        toDeliver
    }

    public enum TeleopPath {
        red,
        blue,
    }

    public void buildPath(TeleopPath allianceColor, Vector2D startPos, Vector2D endPos){

        switch (allianceColor) {
            case red:

                if (startPos.getX() < endPos.getX()){
                    buildTeleopRedDeliver(startPos, endPos);
                }else {
                    buildTeleopRedCollect(startPos, endPos);
                }

                break;
            case blue:

                if (startPos.getX() < endPos.getX()){
                    buildTeleopBlueDeliver(startPos, endPos);
                }else {
                    buildTeleopBlueCollect(startPos, endPos);
                }

                break;
            default:
        }

        pathBuilder(originalPath);

        motionProfile();
    }

    private void buildTeleopBlueCollect(Vector2D startPos, Vector2D targetPos){

        Vector2D throughPosStart = null;
        Vector2D throughPosEnd = null;

        if (startPos.getY() < 60 && startPos.getX() > 0){
            throughPosStart = T1Start;
            throughPosEnd = T1End;
        } else if (startPos.getY() < 120 && startPos.getY() > 60) {
            throughPosStart = T2Start;
            throughPosEnd = T2End;
        } else if (startPos.getY() < 180 && startPos.getY() > 120) {
            throughPosStart = T3Start;
            throughPosEnd = T3End;
        } else if (startPos.getY() < 360 && startPos.getY() > 180) {
            throughPosStart = T4Start;
            throughPosEnd = T4End;
        }

        Vector2D controlForCollectSide = new Vector2D(targetPos.getX() - ((targetPos.getX() - 120)/2)-Math.abs((targetPos.getY() - throughPosStart.getY())/2), throughPosStart.getY());

        if (controlForCollectSide.getX() < 20){
            controlForCollectSide.setX(20);
        }

        Vector2D controlForDeiverySide = new Vector2D(startPos.getX() - ((startPos.getX() - 180)/2)-Math.abs((startPos.getY() - throughPosEnd.getY())/2), throughPosEnd.getY());

        if (controlForDeiverySide.getX() > 300){
            controlForDeiverySide.setX(300);
        }

        buildCurveSegment(startPos, controlForDeiverySide, throughPosEnd);

        buildLineSegment(throughPosEnd, throughPosStart);

        buildCurveSegment(throughPosStart, controlForCollectSide, targetPos);

    }

    private void buildTeleopBlueDeliver(Vector2D startPos, Vector2D targetPos){

        Vector2D throughPosStart = null;
        Vector2D throughPosEnd = null;

        if (startPos.getY() < 60 && startPos.getX() > 0){
            throughPosStart = T1Start;
            throughPosEnd = T1End;
        } else if (startPos.getY() < 120 && startPos.getY() > 60) {
            throughPosStart = T2Start;
            throughPosEnd = T2End;
        } else if (startPos.getY() < 180 && startPos.getY() > 120) {
            throughPosStart = T3Start;
            throughPosEnd = T3End;
        } else if (startPos.getY() < 360 && startPos.getY() > 180) {
            throughPosStart = T4Start;
            throughPosEnd = T4End;
        }

        Vector2D controlForCollectSide = new Vector2D(startPos.getX() - ((startPos.getX() - 120)/2)-Math.abs((startPos.getY() - throughPosStart.getY())/2), throughPosStart.getY());

        if (controlForCollectSide.getX() < 20){
            controlForCollectSide.setX(20);
        }

        Vector2D controlForDeiverySide = new Vector2D(targetPos.getX() - ((targetPos.getX() - 180)/2)-Math.abs((targetPos.getY() - throughPosEnd.getY())/2), throughPosEnd.getY());

        if (controlForDeiverySide.getX() > 300){
            controlForDeiverySide.setX(300);
        }

        buildCurveSegment(startPos, controlForCollectSide, throughPosStart);

        buildLineSegment(throughPosStart, throughPosEnd);

        buildCurveSegment(throughPosEnd, controlForDeiverySide, targetPos);

    }

    private void buildTeleopRedCollect(Vector2D startPos, Vector2D targetPos){

        Vector2D throughPosStart = null;
        Vector2D throughPosEnd = null;

        if (startPos.getY() < 60 && startPos.getX() > 0){
            throughPosEnd = T1Start;
            throughPosStart = T1End;
        } else if (startPos.getY() < 120 && startPos.getY() > 60) {
            throughPosEnd = T2Start;
            throughPosStart = T2End;
        } else if (startPos.getY() < 180 && startPos.getY() > 120) {
            throughPosEnd = T3Start;
            throughPosStart = T3End;
        } else if (startPos.getY() < 240 && startPos.getY() > 180) {
            throughPosEnd = T4Start;
            throughPosStart = T4End;
        }else if (startPos.getY() < 360 && startPos.getY() > 240) {
            throughPosEnd = T5Start;
            throughPosStart = T5End;
        }

        Vector2D controlForCollectSide = new Vector2D(targetPos.getX() - ((targetPos.getX() - 120)/2)-Math.abs((targetPos.getY() - throughPosStart.getY())/2), throughPosStart.getY());

        if (controlForCollectSide.getX() < 20){
            controlForCollectSide.setX(20);
        }

        Vector2D controlForDeiverySide = new Vector2D(startPos.getX() - ((startPos.getX() - 180)/2)-Math.abs((startPos.getY() - throughPosEnd.getY())/2), throughPosEnd.getY());

        if (controlForDeiverySide.getX() > 300){
            controlForDeiverySide.setX(300);
        }

        buildCurveSegment(startPos, controlForDeiverySide, throughPosEnd);

        buildLineSegment(throughPosEnd, throughPosStart);

        buildCurveSegment(throughPosStart, controlForCollectSide, targetPos);


    }

    private void buildTeleopRedDeliver(Vector2D startPos, Vector2D targetPos){

        Vector2D throughPosStart = null;
        Vector2D throughPosEnd = null;

        if (startPos.getY() < 120 && startPos.getX() > 0){
            throughPosStart = T3Start;
            throughPosEnd = T3End;
        } else if (startPos.getY() < 240 && startPos.getY() > 120) {
            throughPosStart = T4Start;
            throughPosEnd = T4End;
        } else if (startPos.getY() < 300 && startPos.getY() > 240) {
            throughPosStart = T5Start;
            throughPosEnd = T5End;
        } else if (startPos.getY() < 360 && startPos.getY() > 300) {
            throughPosStart = T6Start;
            throughPosEnd = T6End;
        }

        Vector2D controlForCollectSide = new Vector2D(startPos.getX() - ((startPos.getX() - 120)/2)-Math.abs((startPos.getY() - throughPosStart.getY())/2), throughPosStart.getY());

        if (controlForCollectSide.getX() < 20){
            controlForCollectSide.setX(20);
        }

        Vector2D controlForDeiverySide = new Vector2D(targetPos.getX() - ((targetPos.getX() - 180)/2)-Math.abs((targetPos.getY() - throughPosEnd.getY())/2), throughPosEnd.getY());

        if (controlForDeiverySide.getX() > 300){
            controlForDeiverySide.setX(300);
        }

        buildCurveSegment(startPos, controlForCollectSide, throughPosStart);

        buildLineSegment(throughPosStart, throughPosEnd);

        buildCurveSegment(throughPosEnd, controlForDeiverySide, targetPos);

    }

}
