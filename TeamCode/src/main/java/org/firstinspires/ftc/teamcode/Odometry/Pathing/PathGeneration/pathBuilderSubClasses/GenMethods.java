package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderMain;

public class GenMethods extends pathBuilderMain {

    public void twoPoints(Vector2D start, Vector2D end, boolean lastSegment){
        buildLineSegment(start, end);

        pathBuilder(originalPath);

        motionProfile();
    }

    public void twoPoints(Vector2D start, Vector2D end){
        buildLineSegment(start, end);
    }

    public void threePoints(Vector2D start, Vector2D control, Vector2D end, boolean lastSegment){
        buildCurveSegment(start, control, end);

        pathBuilder(originalPath);

        motionProfile();
    }

    public void threePoints(Vector2D start, Vector2D control, Vector2D end){
        buildCurveSegment(start, control, end);
    }

    public void fourPoints(Vector2D start, Vector2D control1, Vector2D control2, Vector2D end, boolean lastSegment){
        buildCurveSegment(start, control1, control2, end);

        pathBuilder(originalPath);

        motionProfile();
    }

    public void fourPoints(Vector2D start, Vector2D control1, Vector2D control2, Vector2D end){
        buildCurveSegment(start, control1, control2, end);
    }
    
}
